import cv2
import numpy as np
from ultralytics import YOLO

import pyzed.sl as sl


def main() -> None:
    # Initialize ZED camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        return
    
    # Load YOLOv8 model
    model = YOLO("yolov8n.pt")  # nano model for speed
    # force CPU device to avoid backend engine mismatch (use 'cuda:0' if CUDA is available)
    model.to("cpu")
    
    # Create runtime parameters
    runtime_params = sl.RuntimeParameters()
    image = sl.Mat()
    depth = sl.Mat()
    
    print("Object Detection with ZED Camera")
    print("Press 'q' to quit\n")
    
    while True:
        # Grab frame
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Get left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            
            # Convert to OpenCV format
            frame = image.get_data()
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
            # Run YOLO detection (explicit predictor call + device)
            try:
                results = model.predict(source=frame_rgb, conf=0.45, device="cpu")
            except RuntimeError as exc:
                print("Model inference failed:", exc)
                break
            
            # Process detections
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = result.names[class_id]
                    
                    # Get depth at center
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    depth_value = depth.get_value(center_x, center_y)[1]
                    
                    # Draw bounding box
                    color = (0, 255, 0)
                    cv2.rectangle(frame_rgb, (x1, y1), (x2, y2), color, 2)
                    
                    # Draw label with depth
                    label = f"{class_name} {confidence:.2f} Depth: {depth_value:.2f}m"
                    cv2.putText(
                        frame_rgb, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
                    )
            
            # Display frame
            cv2.imshow("ZED Object Detection", frame_rgb)
            
            # Check for exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
    
    # Cleanup
    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()