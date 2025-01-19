#include "app.cpp"

#include <windows.h>

#include <shellapi.h>
#include <string>

#define TRAY_ICON_ID 1
#define WM_TRAY_ICON (WM_USER + 1)

HINSTANCE hInstance;
NOTIFYICONDATA notifyIconData;
static std::atomic_bool running = { true };

// Function to add tray icon
void AddTrayIcon(HWND hwnd)
{
	memset(&notifyIconData, 0, sizeof(NOTIFYICONDATA));
	notifyIconData.cbSize = sizeof(NOTIFYICONDATA);
	notifyIconData.hWnd = hwnd;
	notifyIconData.uID = TRAY_ICON_ID;
	notifyIconData.uFlags = NIF_ICON | NIF_MESSAGE | NIF_TIP;
	notifyIconData.uCallbackMessage = WM_TRAY_ICON;
	notifyIconData.hIcon = (HICON)LoadImage(NULL, "icon.ico", IMAGE_ICON, 128, 128, LR_LOADFROMFILE);
	strcpy_s(notifyIconData.szTip, "AOG Task Controller");

	Shell_NotifyIcon(NIM_ADD, &notifyIconData);
}

void RemoveTrayIcon()
{
	Shell_NotifyIcon(NIM_DELETE, &notifyIconData);
}

void ShowContextMenu(HWND hwnd)
{
	POINT cursorPos;
	GetCursorPos(&cursorPos);

	HMENU hMenu = CreatePopupMenu();
	AppendMenu(hMenu, MF_STRING, 1, "Exit");

	SetForegroundWindow(hwnd); // Required for the menu to work properly
	int command = TrackPopupMenu(hMenu, TPM_RETURNCMD | TPM_NONOTIFY, cursorPos.x, cursorPos.y, 0, hwnd, NULL);
	DestroyMenu(hMenu);

	if (command == 1) // Exit command
	{
		running = false;
	}
}

// Window procedure to handle messages
LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
		case WM_TRAY_ICON:
			if (lParam == WM_RBUTTONDOWN) // Right-click on tray icon
			{
				ShowContextMenu(hwnd);
			}
			break;

		case WM_DESTROY:
			RemoveTrayIcon();
			PostQuitMessage(0);
			break;

		default:
			return DefWindowProc(hwnd, message, wParam, lParam);
	}
	return 0;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
	WNDCLASS wc = { 0 };
	wc.lpfnWndProc = WndProc;
	wc.hInstance = hInstance;
	wc.lpszClassName = TEXT("AOG-TaskController");
	RegisterClass(&wc);

	HWND hwnd = CreateWindowEx(
	  0, "AOG-TaskController", NULL, 0, 0, 0, 0, 0, HWND_MESSAGE, NULL, hInstance, NULL);
	if (!hwnd)
	{
		return -1;
	}

	AddTrayIcon(hwnd);

	AppTC app;
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
	RemoveTrayIcon();
	return 0;
}
