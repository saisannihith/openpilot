// Test-only adapter for Qualcomm EGL's GBM native display.
// Never preload into the real UI. Uses a render node, not DRM display ownership.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <stddef.h>

struct gbm_device;
extern struct gbm_device *gbm_create_device(int fd);

void *eglGetDisplay(void *native_display) {
  static void *(*real_get_display)(void *) = NULL;
  static struct gbm_device *device = NULL;
  if (!real_get_display) {
    void *egl = dlopen("libEGL.so.1", RTLD_NOW | RTLD_LOCAL);
    if (egl) real_get_display = dlsym(egl, "eglGetDisplay");
  }
  if (!real_get_display) return NULL;
  if (!native_display) {
    if (!device) {
      int fd = open("/dev/dri/renderD128", O_RDWR | O_CLOEXEC);
      if (fd < 0) return NULL;
      device = gbm_create_device(fd);
    }
    native_display = device;
  }
  return real_get_display(native_display);
}
