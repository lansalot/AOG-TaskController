#include "app.hpp"

#include <windows.h>

#include <shellapi.h>
#include <iostream>
#include <string>

#define TRAY_ICON_ID 1
static std::atomic_bool running = { true };

// Window procedure to handle messages
LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
		case WM_CLOSE:
			running = false;
			break;

		default:
			return DefWindowProc(hwnd, message, wParam, lParam);
	}
	return 0;
}

std::vector<std::wstring> ParseCommandLineArguments()
{
	int argc;
	LPWSTR *argv = CommandLineToArgvW(GetCommandLineW(), &argc);
	std::vector<std::wstring> arguments;
	if (argv)
	{
		for (int i = 0; i < argc; ++i)
		{
			arguments.push_back(argv[i]);
		}
		LocalFree(argv);
	}
	return arguments;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
	auto arguments = ParseCommandLineArguments();

	// Print or use the arguments (for debugging or logic)
	for (const auto &arg : arguments)
	{
		// Replace this with logic to handle specific arguments
		std::wcout << L"Argument: " << arg << std::endl;
	}

	WNDCLASS wc = { 0 };
	wc.lpfnWndProc = WndProc;
	wc.hInstance = hInstance;
	wc.lpszClassName = TEXT("AOG-TaskController");
	RegisterClass(&wc);

	HWND hwnd = CreateWindowEx(WS_EX_TOOLWINDOW, wc.lpszClassName, TEXT("AOG-TaskController"), WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 300, 200, NULL, NULL, hInstance, NULL);
	if (!hwnd)
	{
		return -1;
	}
	ShowWindow(hwnd, SW_SHOWMINNOACTIVE); // Little hack: Keep the window hidden, but still allows AOG (or other applications) to gracefully close it

	Application app;
	app.initialize();

	MSG msg;
	while (running)
	{
		while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

		app.update();
	}

	// Clean up
	app.stop();
	return 0;
}
