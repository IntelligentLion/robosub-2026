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

#
# GLShader
#
class GLShader:

    def __init__(self):
        self._program = 0

    @property
    def program(self):
        return self._program

    def load(self, vertex_shader_source: str, fragment_shader_source: str):
        vertex_shader = GLShader._compile(GL_VERTEX_SHADER, vertex_shader_source)
        fragment_shader = GLShader._compile(GL_FRAGMENT_SHADER, fragment_shader_source)

        self._program = glCreateProgram()
        
        glAttachShader(self._program, vertex_shader)
        glAttachShader(self._program, fragment_shader)
        glLinkProgram(self._program)

        glDeleteShader(vertex_shader)
        glDeleteShader(fragment_shader)

        success = glGetProgramiv(self._program, GL_LINK_STATUS)

        if not success:
            info = glGetProgramInfoLog(self._program).decode("utf-8", errors="ignore")
            raise RuntimeError(f"Shader link error:\n{info}")
        
        return self._program

    def use(self):
        glUseProgram(self._program)

    def release(self):
        if glIsProgram(self._program):
            glDeleteProgram(self._program)

    #
    # Shader Library
    #
    MVP_VERTEX = """
        #version 330 core
            
        layout(location = 0) in vec3 inPosition;
        uniform mat4 uMVP;
        
        void main() {
            gl_Position = uMVP * vec4(inPosition, 1.0);
        }
    """

    UNIFORM_COLOR_FRAGMENT = """
        #version 330 core

        uniform vec4 uColor;
        out vec4 fragColor;
        
        void main() {
            fragColor = uColor;
        }
    """

    @staticmethod
    def _compile(kind, source: str):
        shader = glCreateShader(kind)
        glShaderSource(shader, source)
        glCompileShader(shader)
        success = glGetShaderiv(shader, GL_COMPILE_STATUS)

        if not success:
            info = glGetShaderInfoLog(shader).decode("utf-8", errors="ignore")
            raise RuntimeError(f"Shader compile error:\n{info}")
        
        return shader
