# In angle_producer.py

import time
import math

def calculate_angles_loop(output_queue):
    """
    Generates a simple, continuous sine wave motion for testing.
    """
    start_time = time.time()
    rate = 2 # 30 Hz

    while True:
        # --- NEW TEST CODE ---
        # Calculate angles using a sine wave based on time
        elapsed_time = time.time() - start_time
        
        # Calculate smooth motion for the first two joints
        angle1 = 2 * math.sin(elapsed_time * 0.5) 
        angle2 = 2 * math.cos(elapsed_time * 0.5)
        angle3 = 2 * math.cos(elapsed_time * 0.5)

        current_angles = [angle1, angle2, angle3]
        output_queue.put(current_angles)
        # --- END NEW TEST CODE ---

        time.sleep(rate)