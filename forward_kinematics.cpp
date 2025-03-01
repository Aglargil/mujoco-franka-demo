#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

  struct timespec t1, t2;
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

  int fps = 200;
  int N = 1000;
  double step_duration = 1.0 / fps;
  struct timespec start_time, end_time;
  double elapsed, remaining;

  for (int i = 0; i < N && !glfwWindowShouldClose(window); i++) {
    clock_gettime(CLOCK_REALTIME, &start_time);

    double c = i * (M_PI / 2) / N;
    for (int j = 0; j < model->nu; j++) {
      data->ctrl[j] = c;
    }

    mj_step(model, data);

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

  mjv_freeScene(&scene);
  mjr_freeContext(&context);
  mj_deleteData(data);
  mj_deleteModel(model);

  return 0;
}