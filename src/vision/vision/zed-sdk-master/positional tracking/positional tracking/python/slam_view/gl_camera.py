###########################################################################
#
# Copyright (c) 2025, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
###########################################################################

import numpy as np

#
# GLCamera
#
class GLCamera:

    def __init__(
        self,
        target=(0.0, 0.0, 0.0),
        distance = 3.5,
        yaw = -18.0,
        pitch = 27.0,
        fov = 60.0,
        aspect = 16 / 9,
        znear = 1,
        zfar = 1000
    ):
        self._target = np.array(target, dtype=np.float32)
        self._distance = float(distance)
        self._yaw = float(yaw)
        self._pitch = float(pitch)
        self._fov = float(fov)
        self._aspect = float(aspect)
        self._znear = float(znear)
        self._zfar = float(zfar)
        self._default_distance = float(distance)
        self._default_yaw = float(yaw)
        self._default_pitch = float(pitch)
        self._default_target = np.array(target, dtype=np.float32)

        self._projection = np.identity(4, dtype=np.float32)
        self._view = np.identity(4, dtype=np.float32)

        self._update_projection()
        self._update_view()

    @property
    def projection(self) -> np.ndarray:
        return self._projection.T

    @property
    def view(self) -> np.ndarray:
        return self._view.T

    def resize_viewport(self, aspect):
        self._aspect = aspect
        self._update_projection()

    def zoom(self, delta):
        self._distance -= delta
        self._distance = max(self._distance, 2)
        self._update_view()

    def rotate(self, yaw_delta, pitch_delta):
        self._yaw += yaw_delta
        self._pitch += pitch_delta
        self._pitch = max(-89, min(self._pitch, 89))
        self._update_view()

    def pan(self, delta_x, delta_y):
        forward = np.zeros(3, dtype=np.float32)
        right = np.zeros(3, dtype=np.float32)
        up = np.array([0, 1, 0], dtype=np.float32)
        eye = np.zeros(3, dtype=np.float32)

        pitch_rad = np.deg2rad(self._pitch)
        yaw_rad = np.deg2rad(self._yaw)

        eye[0] = self._target[0] + self._distance * np.cos(pitch_rad) * np.sin(yaw_rad)
        eye[1] = self._target[1] + self._distance * np.sin(pitch_rad)
        eye[2] = self._target[2] + self._distance * np.cos(pitch_rad) * np.cos(yaw_rad)

        forward = self._target - eye
        forward /= np.linalg.norm(forward)

        right = np.cross(forward, up)
        right /= np.linalg.norm(right)

        pan_up = np.cross(right, forward)
        pan_up /= np.linalg.norm(pan_up)

        self._target += -right * delta_x + pan_up * delta_y
        self._update_view()

    def reset(self):
        self._target = self._default_target.copy()
        self._distance = self._default_distance
        self._yaw = self._default_yaw
        self._pitch = self._default_pitch
        self._update_view()

    def _update_projection(self):
        fov_rad = np.deg2rad(self._fov)
        f = 1.0 / np.tan(0.5 * fov_rad)
        nf = 1.0 / (self._znear - self._zfar)

        self._projection.fill(0.0)

        self._projection[0, 0] = f / self._aspect
        self._projection[1, 1] = f
        self._projection[2, 2] = (self._znear + self._zfar) * nf
        self._projection[2, 3] = -1.0
        self._projection[3, 2] = 2.0 * self._znear * self._zfar * nf
        self._projection[3, 3] = 0.0

    def _update_view(self):
        pitch_rad = np.deg2rad(self._pitch)
        yaw_rad = np.deg2rad(self._yaw)

        # Compute eye position
        eye = np.zeros(3, dtype=np.float32)
        eye[0] = self._target[0] + self._distance * np.cos(pitch_rad) * np.sin(yaw_rad)
        eye[1] = self._target[1] + self._distance * np.sin(pitch_rad)
        eye[2] = self._target[2] + self._distance * np.cos(pitch_rad) * np.cos(yaw_rad)

        # Forward vector
        forward = self._target - eye
        forward /= np.linalg.norm(forward)

        # Right vector
        up = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)

        # Recompute up to ensure orthogonality
        up = np.cross(right, forward)
        up /= np.linalg.norm(up)

        # Update view matrix
        self._view[0, 0] = right[0]
        self._view[0, 1] = up[0]
        self._view[0, 2] = -forward[0]
        self._view[0, 3] = 0.0

        self._view[1, 0] = right[1]
        self._view[1, 1] = up[1]
        self._view[1, 2] = -forward[1]
        self._view[1, 3] = 0.0

        self._view[2, 0] = right[2]
        self._view[2, 1] = up[2]
        self._view[2, 2] = -forward[2]
        self._view[2, 3] = 0.0

        self._view[3, 0] = -np.dot(right, eye)
        self._view[3, 1] = -np.dot(up, eye)
        self._view[3, 2] = np.dot(forward, eye)
        self._view[3, 3] = 1.0
