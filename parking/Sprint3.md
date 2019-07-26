# Sprint 3

Our top priority is to implement and integrate sensor data fusion to achieve smooth driving. Than we integrate our path planning approaches and select the better one.

In order to have a reliable working demo in the end we will keep it simple and manually select the goal position.

In the final report we are supposed to sketch out an architecture that unifies our parking application with the platooning AF3 model.

## Tasks
- Prepare Demo (Scenario, Exhausting tests)
- Simple collision avoidance
- Presentation (Slides, Topics, Task distribution)
- Decision Custom vs. ROS Navstack:
  - Our custom implementation:
    - Finish data analysis. (Julia)
    - Check and tune positioning accuracy.
    - The Follower should follow multiple splines for more complex paths.
    - Integrate path planning from julia
  - ROS Navstack:
    - fix driver
    - what is our accomplishment?
    - Tune

## Done
- Implemented and check Kalman filter
- Aligned IMU and LIDAR frames
- Calibrated steering
- Checked and calibrated constants like the wheel base
- Improved the Follower code
- Created a Launch file to simplify and speed up testing
- Recorded some data (LIDAR position, IMU and LIDAR orientation and Car's velocity controller step response)
- Test smooth driving
- Smooth driving demo for 04.07.
- The Follower should follow splines again.
- Improve the Kalman filter implementation to have on demand update steps when new sensor data arrives. (Test pending)
- Path plotting node for visualization
- Test the Kalman filter implementation with on demand update steps when new sensor data arrives.
- Tune PID values in the velocity controller.

## Upcoming
- Emergency braking --- Finish high priority stuff first
- Final report --- Deadline on 20.08.2019

## Presentations
- 27.06.
  - Deep Dive: SLAM, Marcel
  - State: Daniel
- 04.07.
  - Deep Dive: Sensor fusion, Hongtao
  - State: Marcel
- 11.07.
  - Deep Dive: Kinematic model, Jannik
  - State: Julia
- 18.07.
  - **Only Demo**
- 25.07.
  - **Final Presentation**