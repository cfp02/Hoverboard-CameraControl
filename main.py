"""
LeafBot Vision GUI:
- GUI application with RTSP stream connection
- Serial communication with ESP32
- Person detection with YOLO (optional)
- Real-time video display with detection overlays

Usage:
  python main.py
"""

import argparse
import time
import threading
import queue
from typing import Optional, Tuple
import serial
import serial.tools.list_ports

import cv2
import numpy as np
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

# --- Detector Abstractions ----------------------------------------------------

class Detection:
    """Simple container for one detection."""
    def __init__(self, xyxy: Tuple[int,int,int,int], conf: float, cls_name: str):
        self.xyxy = xyxy  # (x1,y1,x2,y2) in pixels
        self.conf = conf
        self.cls_name = cls_name

class BaseDetector:
    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        raise NotImplementedError

# ---- YOLO (Ultralytics) ------------------------------------------------------

class YoloDetector(BaseDetector):
    def __init__(self, model_name: str = "yolov8n.pt", device: str = "cpu", conf_thres: float = 0.4):
        from ultralytics import YOLO
        self.model = YOLO(model_name)  # auto-downloads
        self.conf_thres = conf_thres
        self.device = device
        # Restrict to person class id if the model is COCO-trained (class 0 in COCO)
        self.person_names = set(["person"])  # robust to custom class names

    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        # Ultralytics expects RGB
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results = self.model.predict(frame_rgb, verbose=False, conf=self.conf_thres, device=self.device)
        dets: list[Detection] = []
        if not results:
            return dets
        r = results[0]
        if r.boxes is None:
            return dets
        names = r.names if hasattr(r, "names") else {}
        for box in r.boxes:
            cls_id = int(box.cls[0]) if box.cls is not None else -1
            cls_name = names.get(cls_id, str(cls_id))
            if cls_name not in self.person_names:
                continue
            conf = float(box.conf[0]) if box.conf is not None else 0.0
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            dets.append(Detection((x1,y1,x2,y2), conf, cls_name))
        return dets

# --- Utility ------------------------------------------------------------------

def pick_target(detections: list[Detection], strategy: str = "largest") -> Optional[Detection]:
    """Choose one person detection to follow/highlight."""
    if not detections:
        return None
    if strategy == "conf":
        return max(detections, key=lambda d: d.conf)
    return max(detections, key=lambda d: (d.xyxy[2]-d.xyxy[0])*(d.xyxy[3]-d.xyxy[1]))

def compute_errors(bbox: Tuple[int,int,int,int], W: int, H: int, target_h_frac: float = 0.45) -> Tuple[float,float]:
    """Compute control-friendly errors from a bbox."""
    x1, y1, x2, y2 = bbox
    cx = 0.5*(x1 + x2)
    h  = (y2 - y1)
    ex = (cx - (W/2)) / (W/2 + 1e-6)
    desired_h = target_h_frac * H
    ez = (desired_h - h) / (desired_h + 1e-6)
    return float(ex), float(ez)

def draw_bbox(frame: np.ndarray, det: Detection, ex: float, ez: float, fps: float):
    x1,y1,x2,y2 = det.xyxy
    cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
    label = f"{det.cls_name} {det.conf:.2f} | ex={ex:+.2f} ez={ez:+.2f}"
    cv2.putText(frame, label, (x1, max(0,y1-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
    cv2.putText(frame, f"FPS: {fps:.1f}", (8,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)

# --- Serial Manager (Compartmentalized) ---------------------------------------

class SerialManager:
    """Manages serial communication in a thread-safe manner."""
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.conn = None
        self.lock = threading.Lock()
        self.is_connected = False
        
    def connect(self) -> bool:
        """Connect to serial port. Returns True if successful."""
        with self.lock:
            if self.is_connected:
                return True
            try:
                self.conn = serial.Serial(self.port, self.baudrate, timeout=1)
                self.is_connected = True
                return True
            except Exception:
                self.conn = None
                self.is_connected = False
                return False
    
    def disconnect(self):
        """Disconnect from serial port."""
        with self.lock:
            if self.conn:
                try:
                    self.conn.close()
                except:
                    pass
                self.conn = None
            self.is_connected = False
    
    def write(self, data: bytes) -> bool:
        """Write data to serial port. Returns True if successful."""
        with self.lock:
            if not self.is_connected or not self.conn:
                return False
            try:
                if self.conn.is_open:
                    self.conn.write(data)
                    self.conn.flush()
                    return True
            except Exception:
                self.is_connected = False
                if self.conn:
                    try:
                        self.conn.close()
                    except:
                        pass
                    self.conn = None
            return False
    
    def read_available(self) -> str:
        """Read available data from serial port. Returns empty string if nothing available."""
        with self.lock:
            if not self.is_connected or not self.conn:
                return ""
            try:
                if self.conn.in_waiting > 0:
                    return self.conn.read(self.conn.in_waiting).decode('utf-8', errors='ignore')
            except Exception:
                pass
            return ""

# --- Video Manager (Compartmentalized) ---------------------------------------

class VideoManager:
    """Manages video stream capture."""
    def __init__(self, url: str):
        self.url = url
        self.cap = None
        self.is_connected = False
        
    def connect(self) -> bool:
        """Connect to video stream. Returns True if successful."""
        if self.is_connected:
            return True
        try:
            if self.url.startswith("http://") or self.url.startswith("https://"):
                self.cap = cv2.VideoCapture(self.url)
            else:
                self.cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
                if not self.cap.isOpened():
                    self.cap = cv2.VideoCapture(self.url)
            
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.is_connected = True
                return True
        except Exception:
            pass
        return False
    
    def disconnect(self):
        """Disconnect from video stream."""
        if self.cap:
            try:
                self.cap.release()
            except:
                pass
            self.cap = None
        self.is_connected = False
    
    def read_frame(self) -> Optional[np.ndarray]:
        """Read a frame from the stream. Returns None if failed."""
        if not self.is_connected or not self.cap:
            return None
        try:
            ok, frame = self.cap.read()
            if ok and frame is not None:
                return frame
        except Exception:
            pass
        return None

# --- GUI Application ---------------------------------------------------------

class LeafBotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("LeafBot Vision Control")
        self.root.geometry("1200x700")
        
        # Managers (compartmentalized)
        self.serial_manager = None
        self.video_manager = None
        self.detector = None
        
        # State
        self.is_connected = False
        self.stop_threads = False
        self.video_thread = None
        self.serial_thread = None
        
        # Queues
        self.video_queue = queue.Queue(maxsize=2)
        self.serial_output_queue = queue.Queue()
        
        # FPS tracking
        self.prev_t = time.time()
        self.fps = 0.0
        
        # Serial send throttling
        self.last_serial_send_time = 0
        self.serial_send_interval = 0.05  # 50ms = 20 Hz max
        
        self.setup_ui()
        self.start_periodic_updates()
        
    def setup_ui(self):
        """Set up the user interface."""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # Left panel
        left_panel = ttk.Frame(main_frame)
        left_panel.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        
        # Connection controls
        conn_frame = ttk.LabelFrame(left_panel, text="Connection Settings", padding="10")
        conn_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Label(conn_frame, text="Stream URL:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.rtsp_entry = ttk.Entry(conn_frame, width=40)
        self.rtsp_entry.insert(0, "http://192.168.1.230/stream")
        self.rtsp_entry.grid(row=0, column=1, columnspan=2, sticky=(tk.W, tk.E), pady=5, padx=5)
        
        ttk.Label(conn_frame, text="Serial Port:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.serial_combo = ttk.Combobox(conn_frame, width=25, state="readonly")
        self.serial_combo.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=5, padx=5)
        self.refresh_serial_ports()
        
        refresh_btn = ttk.Button(conn_frame, text="Refresh", command=self.refresh_serial_ports, width=10)
        refresh_btn.grid(row=1, column=2, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection, width=20)
        self.connect_btn.grid(row=2, column=0, columnspan=3, pady=10)
        
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=3, column=0, columnspan=3, pady=5)
        
        # Control sliders
        control_frame = ttk.LabelFrame(left_panel, text="Manual Control", padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        speed_frame = ttk.Frame(control_frame)
        speed_frame.grid(row=0, column=0, padx=10, pady=5)
        ttk.Label(speed_frame, text="Speed").pack()
        self.speed_var = tk.IntVar(value=0)
        self.speed_scale = tk.Scale(speed_frame, from_=-100, to=100, orient=tk.VERTICAL, 
                                    variable=self.speed_var, length=200, command=self.on_slider_change)
        self.speed_scale.pack()
        self.speed_value_label = ttk.Label(speed_frame, text="0")
        self.speed_value_label.pack()
        
        turn_frame = ttk.Frame(control_frame)
        turn_frame.grid(row=0, column=1, padx=10, pady=5)
        ttk.Label(turn_frame, text="Turn").pack()
        self.turn_var = tk.IntVar(value=0)
        self.turn_scale = tk.Scale(turn_frame, from_=-100, to=100, orient=tk.VERTICAL,
                                   variable=self.turn_var, length=200, command=self.on_slider_change)
        self.turn_scale.pack()
        self.turn_value_label = ttk.Label(turn_frame, text="0")
        self.turn_value_label.pack()
        
        # Serial output
        serial_frame = ttk.LabelFrame(left_panel, text="Serial Output", padding="10")
        serial_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        
        self.serial_text = scrolledtext.ScrolledText(serial_frame, width=50, height=15, wrap=tk.WORD)
        self.serial_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.serial_text.config(state=tk.DISABLED)
        
        clear_btn = ttk.Button(serial_frame, text="Clear", command=self.clear_serial_output)
        clear_btn.grid(row=1, column=0, pady=5)
        
        # Right panel - Video
        right_panel = ttk.Frame(main_frame)
        right_panel.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        video_frame = ttk.LabelFrame(right_panel, text="Video Feed", padding="10")
        video_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.video_label = ttk.Label(video_frame, text="No video feed", background="black", foreground="white")
        self.video_label.grid(row=0, column=0)
        
        # Grid weights
        main_frame.columnconfigure(0, weight=0)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
        left_panel.columnconfigure(0, weight=1)
        left_panel.rowconfigure(2, weight=1)
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(0, weight=1)
        conn_frame.columnconfigure(1, weight=1)
        serial_frame.columnconfigure(0, weight=1)
        serial_frame.rowconfigure(0, weight=1)
        
    def refresh_serial_ports(self):
        """Refresh available serial ports."""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.serial_combo['values'] = ports
        if ports and not self.serial_combo.get():
            self.serial_combo.current(0)
    
    def clear_serial_output(self):
        """Clear serial output text."""
        self.serial_text.config(state=tk.NORMAL)
        self.serial_text.delete(1.0, tk.END)
        self.serial_text.config(state=tk.DISABLED)
    
    def append_serial_output(self, text: str):
        """Append text to serial output (thread-safe via queue)."""
        try:
            self.serial_output_queue.put(text, block=False)
        except queue.Full:
            pass
    
    def on_slider_change(self, value=None):
        """Handle slider changes - send serial command."""
        if not self.is_connected or not self.serial_manager:
            return
        
        try:
            speed = self.speed_var.get()
            turn = self.turn_var.get()
            
            # Update labels
            self.speed_value_label.config(text=str(speed))
            self.turn_value_label.config(text=str(turn))
            
            # Throttle sends
            current_time = time.time()
            if current_time - self.last_serial_send_time < self.serial_send_interval:
                return
            
            # Send command
            command = f"S:{speed},T:{turn}\n"
            command_bytes = command.encode('utf-8')
            if self.serial_manager.write(command_bytes):
                self.last_serial_send_time = current_time
                # Debug: optionally log sent commands (uncomment for debugging)
                # self.append_serial_output(f"Sent: {command.strip()}\n")
        except Exception:
            pass  # Silently ignore errors to prevent crashes
    
    def toggle_connection(self):
        """Connect or disconnect."""
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        """Connect to stream and serial."""
        rtsp_url = self.rtsp_entry.get().strip()
        serial_port = self.serial_combo.get()
        
        if not rtsp_url:
            messagebox.showerror("Error", "Please enter a stream URL")
            return
        
        # Connect video
        self.video_manager = VideoManager(rtsp_url)
        if self.video_manager.connect():
            self.append_serial_output(f"Stream connected: {rtsp_url}\n")
        else:
            self.append_serial_output(f"Warning: Could not connect to stream (continuing without video)\n")
            self.video_manager = None
        
        # Connect serial
        if serial_port:
            self.serial_manager = SerialManager(serial_port, 115200)
            if self.serial_manager.connect():
                self.append_serial_output(f"Serial connected: {serial_port}\n")
            else:
                self.append_serial_output(f"Error: Could not connect to serial port\n")
                self.serial_manager = None
        else:
            self.serial_manager = None
        
        if not self.video_manager and not self.serial_manager:
            messagebox.showerror("Error", "Could not connect to stream or serial port")
            return
        
        # Initialize YOLO detector if we have video
        if self.video_manager and self.detector is None:
            try:
                self.append_serial_output("Initializing YOLO detector...\n")
                self.detector = YoloDetector(model_name="yolov8n.pt", device="cpu", conf_thres=0.4)
                self.append_serial_output("YOLO detector initialized.\n")
            except Exception as e:
                self.append_serial_output(f"Warning: Could not initialize detector: {e}\n")
                self.detector = None
        
        # Start threads
        self.is_connected = True
        self.stop_threads = False
        
        if self.video_manager:
            self.video_thread = threading.Thread(target=self.video_loop, daemon=True)
            self.video_thread.start()
        
        if self.serial_manager:
            self.serial_thread = threading.Thread(target=self.serial_loop, daemon=True)
            self.serial_thread.start()
            self.speed_scale.config(state=tk.NORMAL)
            self.turn_scale.config(state=tk.NORMAL)
        
        # Update UI
        self.connect_btn.config(text="Disconnect")
        self.status_label.config(text="Status: Connected", foreground="green")
        self.rtsp_entry.config(state=tk.DISABLED)
        self.serial_combo.config(state=tk.DISABLED)
    
    def disconnect(self):
        """Disconnect from stream and serial."""
        # Stop threads first
        self.stop_threads = True
        self.is_connected = False
        
        # Wait for threads to finish
        if self.video_thread:
            self.video_thread.join(timeout=1.0)
            self.video_thread = None
        if self.serial_thread:
            self.serial_thread.join(timeout=1.0)
            self.serial_thread = None
        
        # Disconnect managers
        if self.video_manager:
            self.video_manager.disconnect()
            self.video_manager = None
        
        if self.serial_manager:
            self.serial_manager.disconnect()
            self.serial_manager = None
        
        # Clear queues
        while not self.video_queue.empty():
            try:
                self.video_queue.get_nowait()
            except:
                pass
        while not self.serial_output_queue.empty():
            try:
                self.serial_output_queue.get_nowait()
            except:
                pass
        
        # Update UI
        self.video_label.config(image='', text="No video feed")
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.rtsp_entry.config(state=tk.NORMAL)
        self.serial_combo.config(state="readonly")
        self.speed_scale.config(state=tk.DISABLED)
        self.turn_scale.config(state=tk.DISABLED)
        self.speed_var.set(0)
        self.turn_var.set(0)
        self.speed_value_label.config(text="0")
        self.turn_value_label.config(text="0")
        self.append_serial_output("Disconnected.\n")
    
    def video_loop(self):
        """Video capture thread."""
        while not self.stop_threads and self.is_connected:
            if not self.video_manager:
                break
            
            frame = self.video_manager.read_frame()
            if frame is None:
                time.sleep(0.1)
                continue
            
            try:
                H, W = frame.shape[:2]
                
                # Run YOLO detection if detector is available
                if self.detector:
                    try:
                        detections = self.detector.detect(frame)
                        target = pick_target(detections, strategy="largest")
                        
                        if target is not None:
                            # Compute control errors (for future use)
                            ex, ez = compute_errors(target.xyxy, W, H, target_h_frac=0.45)
                            # Draw bounding box and info
                            draw_bbox(frame, target, ex, ez, self.fps)
                        else:
                            # No person detected
                            cv2.putText(frame, f"FPS: {self.fps:.1f}", (8,20), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
                            cv2.putText(frame, "No person detected", (8,46), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)
                    except Exception as e:
                        # If detection fails, just show FPS
                        cv2.putText(frame, f"FPS: {self.fps:.1f}", (8,20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
                        cv2.putText(frame, "Detection error", (8,46), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2, cv2.LINE_AA)
                else:
                    # No detector - just show FPS
                    cv2.putText(frame, f"FPS: {self.fps:.1f}", (8,20), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
                
                # Calculate FPS
                now = time.time()
                dt = now - self.prev_t
                if dt > 0:
                    self.fps = 1.0 / dt
                self.prev_t = now
                
                # Resize for display
                display_frame = frame.copy()
                max_w, max_h = 800, 600
                if W > max_w or H > max_h:
                    scale = min(max_w / W, max_h / H)
                    new_w, new_h = int(W * scale), int(H * scale)
                    display_frame = cv2.resize(display_frame, (new_w, new_h))
                
                # Convert to RGB
                display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                
                # Put in queue
                try:
                    if not self.video_queue.full():
                        self.video_queue.put(display_frame, block=False)
                except:
                    pass
            except Exception:
                pass
            
            time.sleep(0.033)
    
    def serial_loop(self):
        """Serial read thread."""
        buffer = ""
        while not self.stop_threads and self.is_connected:
            if not self.serial_manager:
                break
            
            try:
                data = self.serial_manager.read_available()
                if data:
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            self.append_serial_output(line.strip() + '\n')
            except Exception:
                pass
            
            time.sleep(0.01)
    
    def start_periodic_updates(self):
        """Start periodic UI updates from queues."""
        self.update_from_queues()
    
    def update_from_queues(self):
        """Update UI from queues (runs in main thread)."""
        # Update video display
        try:
            if not self.video_queue.empty():
                display_frame = self.video_queue.get(block=False)
                img = Image.fromarray(display_frame)
                img_tk = ImageTk.PhotoImage(image=img)
                self.video_label.config(image=img_tk, text="")
                self.video_label.image = img_tk
        except:
            pass
        
        # Update serial output
        try:
            while not self.serial_output_queue.empty():
                text = self.serial_output_queue.get(block=False)
                self.serial_text.config(state=tk.NORMAL)
                self.serial_text.insert(tk.END, text)
                self.serial_text.see(tk.END)
                self.serial_text.config(state=tk.DISABLED)
        except:
            pass
        
        # Schedule next update
        self.root.after(33, self.update_from_queues)
    
    def on_closing(self):
        """Handle window closing."""
        if self.is_connected:
            self.disconnect()
        self.root.destroy()

# --- Main ---------------------------------------------------------------------

def main():
    root = tk.Tk()
    app = LeafBotGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
