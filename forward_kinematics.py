import mujoco
import mujoco.viewer
import numpy as np
import time

model = mujoco.MjModel.from_xml_path("model/franka_emika_panda/panda_nohand.xml")
data = mujoco.MjData(model)

fps = 200
N = 1000
step_duration = 1.0 / fps


with mujoco.viewer.launch_passive(
    model, data, show_left_ui=False, show_right_ui=False
) as viewer:
    viewer.cam.distance = 3.0
    viewer.cam.azimuth = 45.0
    viewer.cam.elevation = -20.0

    for i in range(N):
        start_time = time.time()

        c = i * (np.pi / 2) / N
        data.ctrl[:] = c

        mujoco.mj_step(model, data)

        viewer.sync()

        elapsed = time.time() - start_time
        remaining = step_duration - elapsed
        if remaining > 0:
            time.sleep(remaining)
