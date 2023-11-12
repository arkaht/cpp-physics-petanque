//
//  application.h
//
#pragma once
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "Math/Vector.h"
#include "Math/Quat.h"
#include "Shape.h"
#include "Body.h"

#include "Renderer/DeviceContext.h"
#include "Renderer/model.h"
#include "Renderer/shader.h"
#include "Renderer/FrameBuffer.h"
#include "Camera.h"

/*
====================================================
Application
====================================================
*/
class Application
{
public:
	Application() : m_isPaused( true ), m_stepFrame( false ) {}
	~Application();

	void Initialize();
	void MainLoop();

	void CreateModelForBody( const Body& body );

	Camera& GetCamera() { return camera; }
	bool IsPaused() const { return m_isPaused; }

private:
	std::vector<const char*> GetGLFWRequiredExtensions() const;

	void InitializeGLFW();
	bool InitializeVulkan();
	void Cleanup();
	void UpdateUniforms();
	void DrawFrame();
	void ResizeWindow( int windowWidth, int windowHeight );
	void MouseMoved( float x, float y );
	void MouseScrolled( float z );
	void Keyboard( int key, int scancode, int action, int modifiers );

	static void OnWindowResized( GLFWwindow* window, int width, int height );
	static void OnMouseMoved( GLFWwindow* window, double x, double y );
	static void OnMouseWheelScrolled( GLFWwindow* window, double x, double y );
	static void OnKeyboard( GLFWwindow* window, int key, int scancode, int action, int modifiers );

private:
	class Scene* scene;
	Camera camera;

	GLFWwindow* glfwWindow;

	DeviceContext deviceContext;

	//
	//	Uniform Buffer
	//
	Buffer m_uniformBuffer;

	//
	//	Model
	//
	Model m_modelFullScreen;
	std::vector< Model* > m_models;	// models for the bodies

	//
	//	Pipeline for copying the offscreen framebuffer to the swapchain
	//
	Shader		m_copyShader;
	Descriptors	m_copyDescriptors;
	Pipeline	m_copyPipeline;

	// User input
	Vec2 m_mousePosition;
	bool m_isPaused;
	bool m_stepFrame;

	std::vector<RenderModel> m_renderModels;

	static const int WINDOW_WIDTH = 1200;
	static const int WINDOW_HEIGHT = 720;

	static const bool m_enableLayers = true;
};

extern Application* application;