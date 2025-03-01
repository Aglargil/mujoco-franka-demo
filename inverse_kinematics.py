import mujoco
import mujoco.viewer
import numpy as np
import time

model = mujoco.MjModel.from_xml_path("model/franka_emika_panda/panda_nohand.xml")
data = mujoco.MjData(model)

fps = 300
N = 3000
step_duration = 1.0 / fps

mujoco.mj_forward(model, data)
position = data.site_xpos[0].copy()  # current position
position_des = np.array([0.76, 0.3, 0.35])  # target position

with mujoco.viewer.launch_passive(
    model, data, show_left_ui=False, show_right_ui=False
) as viewer:
    viewer.cam.distance = 3.0
    viewer.cam.azimuth = 45.0
    viewer.cam.elevation = -20.0

    for i in range(N):
        start_time = time.time()

        position = data.site_xpos[0].copy()

        dX = position_des - position
        distance = np.linalg.norm(dX)

        if distance < 0.02:
            print(f"Finished: {i}")
            print(f"Current position: {position}")
            print(f"Target position: {position_des}")
            break

        # Compute Jacobian matrix
        jacp = np.zeros((3, 7))
        mujoco.mj_jacSite(model, data, jacp, None, 0)

        J = jacp

        # Compute pseudo-inverse of Jacobian matrix J+ = J_T(JJ_T)^-1
        JT = J.T
        try:
            Jinv = JT @ np.linalg.inv(J @ JT)
        except np.linalg.LinAlgError:
            # Add damping factor to enhance numerical stability
            lambda_factor = 0.1
            Jinv = JT @ np.linalg.inv(J @ JT + lambda_factor * np.eye(3))

        # Compute joint angle change dq = J+ * dX
        dq = Jinv @ dX

        # Limit step size to maintain stability
        dq_norm = np.linalg.norm(dq)
        if dq_norm > 0.1:
            dq = dq * 0.1 / dq_norm

        # Update joint angle
        data.ctrl[:7] = data.qpos[:7] + dq[:7]

        mujoco.mj_step(model, data)

        if i % 100 == 0:
            print(f"Iteration {i}: Distance to target: {distance:.6f}")
            print(f"Current position: {position}")

        viewer.sync()

        elapsed = time.time() - start_time
        remaining = step_duration - elapsed
        if remaining > 0:
            time.sleep(remaining)

    print("Inverse kinematics solved")
    print(f"Final position: {data.site_xpos[0]}")
    print(f"Target position: {position_des}")
    print(f"Final error: {np.linalg.norm(data.site_xpos[0] - position_des):.6f}")
