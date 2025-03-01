#include <math.h>
#include <mujoco/mjtnum.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <eigen3/Eigen/Dense>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

mjModel* model = NULL;
mjData* data = NULL;
mjvCamera cam;
mjvOption opt;
mjvScene scene;
mjrContext context;

void render(GLFWwindow* window) {
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);

  mjrRect viewport = {0, 0, width, height};
  mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scene);
  mjr_render(viewport, &scene, &context);

  glfwPollEvents();

  glfwSwapBuffers(window);
}

int main(void) {
  char error[1000] = "Failed to load model";
  model = mj_loadXML("../model/franka_emika_panda/panda_nohand.xml", NULL,
                     error, 1000);
  if (!model) {
    printf("%s\n", error);
    return 1;
  }

  data = mj_makeData(model);

  if (!glfwInit()) {
    mj_deleteData(data);
    mj_deleteModel(model);
    printf("Failed to initialize GLFW\n");
    return 1;
  }

  GLFWwindow* window = glfwCreateWindow(
      1200, 900, "Franka Emika Panda forward kinematics", NULL, NULL);
  if (!window) {
    glfwTerminate();
    mj_deleteData(data);
    mj_deleteModel(model);
    printf("Failed to create GLFW window\n");
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(0);  // disable vertical sync

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scene);
  mjr_defaultContext(&context);
  mjv_makeScene(model, &scene, 2000);
  mjr_makeContext(model, &context, mjFONTSCALE_150);

  cam.distance = 3.0;
  cam.azimuth = 45.0;
  cam.elevation = -20.0;

  // initialize position
  mj_forward(model, data);

  int fps = 300;
  int N = 3000;
  double step_duration = 1.0 / fps;
  struct timespec start_time, end_time;
  double elapsed, remaining;
  double distance;

  Eigen::Vector3d position(data->site_xpos[0], data->site_xpos[1],
                           data->site_xpos[2]);
  Eigen::Vector3d position_des(0.76, 0.3, 0.35);

  for (int i = 0; i < N && !glfwWindowShouldClose(window); i++) {
    clock_gettime(CLOCK_REALTIME, &start_time);

    position = Eigen::Vector3d(data->site_xpos[0], data->site_xpos[1],
                               data->site_xpos[2]);

    Eigen::Vector3d dX = position_des - position;
    distance = dX.norm();

    if (distance < 0.02) {
      printf("Finished: %d\n", i);
      printf("Current position: %f, %f, %f\n", position[0], position[1],
             position[2]);
      printf("Target position: %f, %f, %f\n", position_des[0], position_des[1],
             position_des[2]);
      break;
    }

    // Compute Jacobian matrix
    mjtNum jacp[3 * 7];
    mj_jacSite(model, data, jacp, NULL, 0);
    Eigen::Matrix<mjtNum, 3, 7> J;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 7; j++) {
        J(i, j) = jacp[i * 7 + j];
      }
    }
    // Compute pseudo-inverse of Jacobian matrix J+ = J_T(JJ_T)^-1
    auto JT = J.transpose();
    Eigen::Matrix<mjtNum, 7, 3> Jinv;
    try {
      Jinv = JT * ((J * JT).inverse());
    } catch (const std::exception& e) {
      // Add damping factor to enhance numerical stability
      double lambda_factor = 0.1;
      Jinv = JT *
             (J * JT + lambda_factor * Eigen::Matrix<mjtNum, 3, 3>::Identity())
                 .inverse();
    }

    // Compute joint angle change dq = J+ * dX
    Eigen::Matrix<mjtNum, 7, 1> dq = Jinv * dX;

    // Limit step size to maintain stability
    auto dq_norm = dq.norm();
    if (dq_norm > 0.1) {
      dq = dq * 0.1 / dq_norm;
    }

    // Update joint angles
    for (int j = 0; j < 7; j++) {
      data->ctrl[j] = data->qpos[j] + dq[j];
    }

    mj_step(model, data);

    if (i % 100 == 0) {
      printf("Iteration %d: Distance to target: %f\n", i, distance);
      printf("Current position: %f, %f, %f\n", position[0], position[1],
             position[2]);
    }

    render(window);

    clock_gettime(CLOCK_REALTIME, &end_time);
    elapsed = (end_time.tv_sec - start_time.tv_sec) +
              (end_time.tv_nsec - start_time.tv_nsec) / 1e9;
    remaining = step_duration - elapsed;
    if (remaining > 0) {
      struct timespec sleep_time;
      sleep_time.tv_sec = (time_t)remaining;
      sleep_time.tv_nsec = (long)((remaining - sleep_time.tv_sec) * 1e9);
      nanosleep(&sleep_time, NULL);
    }
  }

  printf("Inverse kinematics solved\n");
  printf("Final position: %f, %f, %f\n", position[0], position[1], position[2]);
  printf("Target position: %f, %f, %f\n", position_des[0], position_des[1],
         position_des[2]);
  printf("Final error: %f\n", distance);

  mjv_freeScene(&scene);
  mjr_freeContext(&context);
  mj_deleteData(data);
  mj_deleteModel(model);

  return 0;
}