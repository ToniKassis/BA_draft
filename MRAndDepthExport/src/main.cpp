// Copyright 2019-2021 Varjo Technologies Oy. All rights reserved.

/* Mixed Reality Example Application
 *
 * - Showcases Varjo MR API features: Camera, data streams, rendering, and more!
 * - Run example and press F1 for help
 */

// Internal includes
#include "Globals.hpp"
#include "AppLogic.hpp"
#include "AppView.hpp"

std::unique_ptr<AppLogic, std::default_delete<AppLogic>> appLogic;
std::unique_ptr<AppView, std::default_delete<AppView>> appView;

// Common main function called from the entry point
void commonMain()
{
    // Instantiate application logic and view
    appLogic = std::make_unique<AppLogic>();
    appView = std::make_unique<AppView>(*appLogic);

    try {
        // Init application
        LOG_DEBUG("Initializing application..");
        if (appView->init()) {
            // Enter the main loop
            LOG_DEBUG("Running application..");
            appView->run();
        } else {
            LOG_ERROR("Initializing application failed.");
        }
    } catch (const std::runtime_error& e) {
        LOG_ERROR("Critical error caught: %s", e.what());
    }

    // Deinitialize client app
    LOG_DEBUG("Deinitializing application..");
    appView.reset();
    appLogic.reset();

    // Exit successfully
    LOG_INFO("Done!");
}

BOOL WINAPI ConsoleHandler(DWORD CEvent)
{
    switch (CEvent)
    {
    case CTRL_C_EVENT:
        appView.reset();
        appLogic.reset();
        break;
    case CTRL_BREAK_EVENT:
        appView.reset();
        appLogic.reset();
        break;
    case CTRL_CLOSE_EVENT:
        appView.reset();
        appLogic.reset();
        break;
    case CTRL_LOGOFF_EVENT:
        appView.reset();
        appLogic.reset();
        break;
    case CTRL_SHUTDOWN_EVENT:
        appView.reset();
        appLogic.reset();
        break;

    }
    return TRUE;
}

// Console application entry point
int main(int argc, char** argv)
{
    if (SetConsoleCtrlHandler(
        (PHANDLER_ROUTINE)ConsoleHandler, TRUE) == FALSE)
    {
        // unable to install handler... 
        // display message to the user
        printf("Unable to install handler!\n");
        return -1;
    }

    // Call common main function
    commonMain();

    // Application finished
    return EXIT_SUCCESS;
}

// Windows application entry point
int __stdcall wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow)
{
    int argc = 0;
    LPWSTR* args = CommandLineToArgvW(pCmdLine, &argc);

    bool console = false;
    for (int i = 0; i < argc; i++) {
        if (std::wstring(args[i]) == (L"--console")) {
            console = true;
        }
    }

    if (console) {
        if (AttachConsole(ATTACH_PARENT_PROCESS) || AllocConsole()) {
            errno_t err;
            FILE* stream = nullptr;
            err = freopen_s(&stream, "CONOUT$", "w", stdout);
            err = freopen_s(&stream, "CONOUT$", "w", stderr);
        }
    }

    if (SetConsoleCtrlHandler(
        (PHANDLER_ROUTINE)ConsoleHandler, TRUE) == FALSE)
    {
        // unable to install handler... 
        // display message to the user
        printf("Unable to install handler!\n");
        return -1;
    }

    // Call common main function
    commonMain();

    // Application finished
    return EXIT_SUCCESS;
}
