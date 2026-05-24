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

from OpenGL.GL import *

from .gl_shader import GLShader

#
# Shaders
#
POINT_CLOUD_VERTEX_SHADER = """
    #version 330 core
    
    layout(location = 0) in vec4 in_VertexRGBA;
    uniform mat4 u_mvpMatrix;
    out vec3 b_color;

    vec4 decomposeFloat(const in float value) {
        uint rgbaInt = floatBitsToUint(value);
        uint bIntValue = (rgbaInt / 256U / 256U) % 256U;
        uint gIntValue = (rgbaInt / 256U) % 256U;
        uint rIntValue = (rgbaInt) % 256U;

        return vec4(bIntValue / 255.0f, gIntValue / 255.0f, rIntValue / 255.0f, 1.0);
    }

    void main() {
        // Decompose the 4th channel of the XYZRGBA buffer to retrieve the color of the point (1float to 4uint)
        b_color = decomposeFloat(in_VertexRGBA.a).xyz;
        gl_Position = u_mvpMatrix * vec4(in_VertexRGBA.xyz, 1);
    };
"""

POINT_CLOUD_FRAGMENT_SHADER = """
    #version 330 core

    in vec3 b_color;
    layout(location = 0) out vec4 outColor;
    
    void main() {
        outColor = vec4(b_color, 0.9);
    }
"""

#
# PointCloud
#
class PointCloud:
    
    def __init__(self, mat):
        self._mat = mat
        self._vao = 0
        self._vbo = 0
        self._mvp_loc = 0
        self._count = 0
        self._shader = GLShader()

    def draw(self, mvp):
        self._push_to_gpu()

        if self._count == 0:
            return

        self._shader.use()
        glUniformMatrix4fv(self._mvp_loc, 1, GL_TRUE, mvp)
        glBindVertexArray(self._vao)
        glDrawArrays(GL_POINTS, 0, self._count)
        glBindVertexArray(0)
        glUseProgram(0)

    def release(self):
        if glIsBuffer(self._vbo):
            glDeleteBuffers(1, [self._vbo])
        
        if glIsVertexArray(self._vao):
            glDeleteVertexArrays(1, [self._vao])

        self._shader.release()

    def _push_to_gpu(self):
        self._create_resources_if_necessary()

        flattened = self._mat.get_data().reshape(-1, 4)
        self._count = flattened.shape[0]

        glBindBuffer(GL_ARRAY_BUFFER, self._vbo)
        glBufferData(GL_ARRAY_BUFFER, flattened.nbytes, flattened, GL_DYNAMIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

    def _create_resources_if_necessary(self):
        if glIsVertexArray(self._vao):
            return

        self._vao = glGenVertexArrays(1)
        self._vbo = glGenBuffers(1)

        glBindVertexArray(self._vao)
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo)
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 16, ctypes.c_void_p(0))
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindVertexArray(0)

        self._shader.load(POINT_CLOUD_VERTEX_SHADER, POINT_CLOUD_FRAGMENT_SHADER)
        self._mvp_loc = glGetUniformLocation(self._shader.program, "u_mvpMatrix")
        
