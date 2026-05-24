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
from OpenGL.GL import *

#
# Simple3DObject
#
class Simple3DObject:

    def __init__(self, drawing_type=GL_TRIANGLES, is_static=False):
        self._drawing_type = drawing_type
        self._is_static = is_static
        self._positions = [] # list of (x, y, z)
        self._colors = []    # list of (r, g, b, a)
        self._vao = 0
        self._vbo_positions = 0
        self._vbo_colors = 0
        self._is_dirty = False

    def add_point(self, position, color=(1.0, 1.0, 1.0, 1.0)):
        self._positions.append(position)
        self._colors.append(color)
        self._is_dirty = True

    def clear(self):
        self._positions.clear()
        self._colors.clear()
        self._is_dirty = True

    def draw(self):
        self._push_to_gpu()

        glBindVertexArray(self._vao)
        glDrawArrays(self._drawing_type, 0, len(self._positions))
        glBindVertexArray(0)

    def release(self):
        if glIsBuffer(self._vbo_positions):
            glDeleteBuffers(1, [self._vbo_positions])
        
        if glIsBuffer(self._vbo_colors):
            glDeleteBuffers(1, [self._vbo_colors])

        if glIsVertexArray(self._vao):
            glDeleteVertexArrays(1, [self._vao])

    def _push_to_gpu(self):
        self._create_resources_if_necessary()

        if not self._is_dirty:
            return

        positions = np.array(self._positions, dtype=np.float32)
        colors = np.array(self._colors, dtype=np.float32)

        glBindVertexArray(self._vao)
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo_positions)
        glBufferData(GL_ARRAY_BUFFER, positions.nbytes, positions, GL_STATIC_DRAW if self._is_static else GL_DYNAMIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo_colors)
        glBufferData(GL_ARRAY_BUFFER, colors.nbytes, colors, GL_STATIC_DRAW if self._is_static else GL_DYNAMIC_DRAW)
        glBindVertexArray(0)
        
        self._is_dirty = False

    def _create_resources_if_necessary(self):
        if glIsVertexArray(self._vao):
            return
        
        self._vao = glGenVertexArrays(1)
        glBindVertexArray(self._vao)
        self._vbo_positions = glGenBuffers(1)
        self._vbo_colors = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo_positions)
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, False, 0, None)
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo_colors)
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(1, 4, GL_FLOAT, False, 0, None)
        glBindVertexArray(0)
