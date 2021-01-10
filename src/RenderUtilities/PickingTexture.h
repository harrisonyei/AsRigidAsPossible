#include "BufferObject.h"
#include "Shader.h"
#include "Texture.h"

class PickingTexture
{
public:
	bool Init(unsigned int WindowWidth, unsigned int WindowHeight)
	{
		// Create the FBO
		glGenFramebuffers(1, &fbo.fbo);
		glBindFramebuffer(GL_FRAMEBUFFER, fbo.fbo);

		// Create the texture object for the primitive information buffer
		glGenTextures(1, &fbo.textures[0]);
		glBindTexture(GL_TEXTURE_2D, fbo.textures[0]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, WindowWidth, WindowHeight, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, NULL);
		//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, WindowWidth, WindowHeight,
		//	0, GL_RGB, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
			fbo.textures[0], 0);

		// Disable reading to avoid problems with older GPUs
		glReadBuffer(GL_NONE);

		glDrawBuffer(GL_COLOR_ATTACHMENT0);

		// Verify that the FBO is correct
		GLenum Status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

		if (Status != GL_FRAMEBUFFER_COMPLETE) {
			printf("FB error, status: 0x%x\n", Status);
			return false;
		}

		// Restore the default framebuffer
		glBindTexture(GL_TEXTURE_2D, 0);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		return true;
	}
	bool Resize(unsigned int WindowWidth, unsigned int WindowHeight)
	{
		// Create the FBO
		glBindFramebuffer(GL_FRAMEBUFFER, fbo.fbo);

		// Create the texture object for the primitive information buffer
		glBindTexture(GL_TEXTURE_2D, fbo.textures[0]);
		//glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, WindowWidth, WindowHeight,
		//	0, GL_RED, GL_FLOAT, NULL);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, WindowWidth, WindowHeight, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, NULL);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
			fbo.textures[0], 0);

		// Disable reading to avoid problems with older GPUs
		glReadBuffer(GL_NONE);
		glDrawBuffer(GL_COLOR_ATTACHMENT0);

		// Verify that the FBO is correct
		GLenum Status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

		if (Status != GL_FRAMEBUFFER_COMPLETE) {
			printf("FB error, status: 0x%x\n", Status);
			return false;
		}

		// Restore the default framebuffer
		glBindTexture(GL_TEXTURE_2D, 0);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		return true;
	}
	void EnableWriting(){glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo.fbo);}
	void DisableWriting(){glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);}
	GLuint ReadPixel(unsigned int x, unsigned int y)
	{
		glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo.fbo);
		glReadBuffer(GL_COLOR_ATTACHMENT0);

		GLuint Pixel;
		glReadPixels(x, y, 1, 1, GL_RED_INTEGER, GL_UNSIGNED_INT, &Pixel);

		glReadBuffer(GL_NONE);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

		return Pixel;
	}

private:
	FBO fbo;

	//GLuint m_fbo;
	//GLuint m_pickingTexture;
};
