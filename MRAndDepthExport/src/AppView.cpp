// Copyright 2019-2021 Varjo Technologies Oy. All rights reserved.

#include "AppView.hpp"

#include <unordered_map>
#include <tchar.h>
#include <iostream>
#include <fstream>
#include <imgui_internal.h>
#include <map>
#include <chrono>

// Application title text
#define APP_TITLE_TEXT "Varjo Mixed Reality Example"
#define APP_COPYRIGHT_TEXT "(C) 2019-2021 Varjo Technologies"

// Enable debug frame timing
#define DEBUG_FRAME_TIMING 0

// VarjoExamples namespace contains simple example wrappers for using Varjo API features.
// These are only meant to be used in SDK example applications. In your own application,
// use your own production quality integration layer.
using namespace VarjoExamples;

namespace
{
//! Action info structure
struct ActionInfo {
    std::string name;  //!< Action name
    int keyCode;       //!< Shortcut keycode
    std::string help;  //!< Help string
};

// Key shortcut mapping
std::unordered_map<int, AppView::Action> c_keyMappings;

// Action mappings
// clang-format off
static const std::unordered_map<AppView::Action, ActionInfo> c_actions = {
    {AppView::Action::None,                          {"None",                            -1,         "--    (no action)"}},
    {AppView::Action::Quit,                          {"Quit",                            VK_ESCAPE,  "ESC   Quit"}},
    {AppView::Action::Reset,                         {"Reset",                           -1,         "--    Reset all settings"}},
    {AppView::Action::Help,                          {"Help",                            VK_F1,      "F1    Print help"}},
    {AppView::Action::PrintStreamConfigs,            {"PrintStreamConfigs",              VK_F2,      "F2    Fetch and print stream configs"}},
    {AppView::Action::ToggleBufferHandlingMode,      {"ToggleBufferHandlingMode",        VK_F9,      "F9    Toggle buffer handling mode"}},
    {AppView::Action::ToggleStreamColorYUV,          {"ToggleStreamColorYUV",            VK_DOWN,    "DOWN  Toggle stream COLOR: YUV"}},
    {AppView::Action::ToggleReactConnectionEvents,   {"ToggleReactConnectionEvents",     'C',        "C     Toggle MR availability event handling"}},
};
// clang-format on

// Window client area margin
constexpr int c_windowMargin = 8;

// Window client area size and log height
constexpr glm::ivec2 c_windowClientSize(800, 900);
constexpr int c_logHeight = 332;

}  // namespace

AppView::AppView(AppLogic& logic)
    : m_logic(logic)
{
    // Fill in key mappings
    for (auto& ai : c_actions) {
        c_keyMappings[ai.second.keyCode] = ai.first;
    }

    // Present UI with vsync OFF (We sync to Varjo API instead).
    constexpr bool c_vsync = false;

    // Create user interface instance
    m_ui = std::make_unique<UI>(std::bind(&AppView::onFrame, this, std::placeholders::_1),    //
        std::bind(&AppView::onKeyPress, this, std::placeholders::_1, std::placeholders::_2),  //
        _T(APP_TITLE_TEXT), c_windowClientSize.x, c_windowClientSize.y, c_vsync);

    // Set log function
    LOG_INIT(std::bind(&UI::writeLogEntry, m_ui.get(), std::placeholders::_1, std::placeholders::_2), LogLevel::Info);

    LOG_INFO(APP_TITLE_TEXT);
    LOG_INFO(APP_COPYRIGHT_TEXT);
    LOG_INFO("-------------------------------");

    // Create contexts
    m_context = std::make_unique<GfxContext>(m_ui->getWindowHandle());

    // Additional ImgUi setup
    auto& io = ImGui::GetIO();

    // Disable storing UI ini file
    io.IniFilename = NULL;
}

AppView::~AppView()
{
    // Deinit logger
    LOG_DEINIT();

    // Free UI
    m_ui.reset();
}

bool AppView::init()
{
    if (!m_logic.init(*m_context)) {
        LOG_ERROR("Initializing application failed.");
        return false;
    }

    // Reset states
    m_uiState = {};
    AppState appState = m_logic.getState();

    // Force set initial state
    m_logic.setState(appState, true);

    return true;
}

void AppView::run()
{
    LOG_DEBUG("Entering main loop.");

    // Run UI main loop
    m_ui->run();
}

bool AppView::onFrame(UI& ui)
{
    // Check UI instance
    if (&ui != m_ui.get()) {
        return false;
    }

    // Check for quit
    if (m_uiState.quitRequested) {
        LOG_INFO("Quit requested.");
        return false;
    }

#if DEBUG_FRAME_TIMING
    static std::chrono::nanoseconds maxDuration{0};
    static std::chrono::nanoseconds totDuration{0};
    static int frameCount = 0;
    const auto frameStartTime = std::chrono::high_resolution_clock::now();
#endif

    // Check for varjo events
    m_logic.checkEvents();

    // Update state to logic state
    updateUI();

    // Update application logic
    m_logic.update();

#if DEBUG_FRAME_TIMING
    // Frame timing
    const auto frameEndTime = std::chrono::high_resolution_clock::now();
    const auto frameDuration = frameEndTime - frameStartTime;
    totDuration += frameDuration;
    maxDuration = std::max(frameDuration, maxDuration);
    frameCount++;

    // Report frame timing
    static const std::chrono::seconds c_frameReportInterval(1);
    if (totDuration >= c_frameReportInterval) {
        LOG_INFO("Timing: frames=%d, fps=%f, avg=%f ms, max=%f ms, tot=%f ms", frameCount, 1e9f * frameCount / totDuration.count(),
            0.000001 * totDuration.count() / frameCount, 0.000001 * maxDuration.count(), 0.000001 * totDuration.count());
        maxDuration = {0};
        totDuration = {0};
        frameCount = 0;
    }
#endif

    // Return true to continue running
    return true;
}

bool AppView::onAction(Action actionType, AppState& appState)
{
    Action action = static_cast<Action>(actionType);
    if (c_actions.count(action) == 0) {
        LOG_ERROR("Unknown not found: %d", actionType);
        return false;
    }

    if (action != Action::None) {
        LOG_INFO("Action: %s", c_actions.at(action).name.c_str());
    }

    // App state dirty flag
    bool stateDirty = false;

    // Handle input actions
    switch (action) {
        case Action::None: {
            // Ignore
        } break;

        case Action::Quit: {
            m_uiState.quitRequested = true;
        } break;

        case Action::Help: {
            LOG_INFO("\nKeyboard Shortcuts:\n");
            std::multimap<int, AppView::Action> sortedShortcuts;
            for (const auto& ai : c_actions) {
                if (ai.second.keyCode > 0) {
                    sortedShortcuts.insert(std::make_pair(ai.second.keyCode, ai.first));
                }
            }
            for (const auto& it : sortedShortcuts) {
                if (it.second != Action::None) {
                    LOG_INFO("  %s", c_actions.at(it.second).help.c_str());
                }
            }
            LOG_INFO("");
        } break;

        case Action::Reset: {
            appState.options = {};
            stateDirty = true;
        } break;

        case Action::PrintStreamConfigs: {
            m_logic.getStreamer().printStreamConfigs();
        } break;

        case Action::ToggleBufferHandlingMode: {
            appState.options.delayedBufferHandlingEnabled = !appState.options.delayedBufferHandlingEnabled;
            stateDirty = true;
        } break;

        case Action::ToggleStreamColorYUV: {
            appState.options.dataStreamColorEnabled = !appState.options.dataStreamColorEnabled;
            stateDirty = true;
        } break;

        case Action::ToggleReactConnectionEvents: {
            appState.options.reactToConnectionEvents = !appState.options.reactToConnectionEvents;
            stateDirty = true;
        } break;

        default: {
            // Ignore unknown action
            LOG_ERROR("Unknown action: %d", action);
        } break;
    }

    return stateDirty;
}

void AppView::onKeyPress(UI& ui, int keyCode)
{
    if (m_uiState.anyItemActive) {
        // Ignore key handling if UI items active
        return;
    }

    // Check for input action
    Action action = Action::None;
    if (c_keyMappings.count(keyCode)) {
        action = c_keyMappings.at(keyCode);
    }

    // Get current state
    auto appState = m_logic.getState();

    // Handle action
    const bool stateDirty = onAction(action, appState);

    // Update state if changed
    if (stateDirty) {

        // Set app logic state
        m_logic.setState(appState, false);
    }
}

void AppView::updateUI()
{
    // --- UI helper definitions --->
#define _VSPACE ImGui::Dummy(ImVec2(0.0f, 8.0f));

#define _HSPACE                       \
    ImGui::Dummy(ImVec2(8.0f, 8.0f)); \
    ImGui::SameLine();

#define _SEPARATOR      \
    _VSPACE;            \
    ImGui::Separator(); \
    _VSPACE;

#define _PUSHSTYLE_ALPHA(X) ImGui::PushStyleVar(ImGuiStyleVar_Alpha, X);

#define _POPSTYLE ImGui::PopStyleVar();

#define _PUSHDISABLEDIF(C)                                  \
    if (C) {                                                \
        _PUSHSTYLE_ALPHA(0.5f);                             \
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true); \
    }

#define _POPDISABLEDIF(C)     \
    if (C) {                  \
        _POPSTYLE;            \
        ImGui::PopItemFlag(); \
    }
    // <--- UI helper definitions ---

    // Update from logic state
    AppState appState = m_logic.getState();

    // VST Post Process window
    ImGui::Begin(APP_TITLE_TEXT);

    // Set initial size and pos
    {
        const float m = static_cast<float>(c_windowMargin);
        const float w = static_cast<float>(c_windowClientSize.x);
        const float h = static_cast<float>(c_windowClientSize.y - c_logHeight);
        ImGui::SetWindowPos(ImVec2(m, m), ImGuiCond_FirstUseEver);
        ImGui::SetWindowSize(ImVec2(w - 2 * m, h - 2 * m), ImGuiCond_FirstUseEver);
    }

#define _TAG "##appgeneric"

    {
        _VSPACE;
        _HSPACE;
        ImGui::BeginGroup();

        if (ImGui::Button("Reset" _TAG)) {
            onAction(Action::Reset, appState);
        }

        ImGui::SameLine();
        if (ImGui::Button("Help" _TAG)) {
            onAction(Action::Help, appState);
        }

        ImGui::EndGroup();
    }
#undef _TAG

    _SEPARATOR;

#define _TAG "##mixedreality"
    {
        {
            ImGui::Text("Mixed Reality:");
            _VSPACE;
            _HSPACE;
            ImGui::BeginGroup();
            ImGui::Checkbox("React MR events" _TAG, &appState.options.reactToConnectionEvents);
            ImGui::EndGroup();
        }
    }
#undef _TAG

    _SEPARATOR;

#define _TAG "##datastreams"
    {
        ImGui::Text("Data Streaming:");
        _VSPACE;
        _HSPACE;
        ImGui::BeginGroup();

        if (ImGui::Button("Print Configs" _TAG)) {
            onAction(Action::PrintStreamConfigs, appState);
        }

        ImGui::SameLine();
        _HSPACE;
        ImGui::Checkbox("Stream: Color" _TAG, &appState.options.dataStreamColorEnabled);

        ImGui::SameLine();
        _HSPACE;
        ImGui::Checkbox("Delayed handling" _TAG, &appState.options.delayedBufferHandlingEnabled);

        _VSPACE;
        ImGui::Text("Status: %s", m_logic.getStreamer().getStatusLine().c_str());

        ImGui::EndGroup();
    }
#undef _TAG

    _SEPARATOR;

    {
        if (!appState.general.mrAvailable) {
            const auto colTxt = ImGui::GetStyleColorVec4(ImGuiCol_Text);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(colTxt.x, colTxt.y, colTxt.z * 0.2f, colTxt.w));
        }
        ImGui::Text(appState.general.mrAvailable ? "Status: Mixed reality features available." : "Warning: Mixed Reality features not available.");
        if (!appState.general.mrAvailable) {
            ImGui::PopStyleColor(1);
        }

        ImGui::Text("Frame timing: %.3f fps / %.3f ms / %.3f s / %d frames",  //
            ImGui::GetIO().Framerate,                                         //
            1000.0f / ImGui::GetIO().Framerate,                               //
            appState.general.frameTime, appState.general.frameCount);
    }

    // End main window
    ImGui::End();

    // Log window
    {
        ImGui::Begin("Log");

        // Set initial size and pos
        {
            const float m = static_cast<float>(c_windowMargin);
            const float w = static_cast<float>(c_windowClientSize.x);
            const float h0 = static_cast<float>(c_windowClientSize.y - c_logHeight);
            const float h1 = static_cast<float>(c_logHeight);
            ImGui::SetWindowPos(ImVec2(m, h0), ImGuiCond_FirstUseEver);
            ImGui::SetWindowSize(ImVec2(w - 2 * m, h1 - m), ImGuiCond_FirstUseEver);
        }

        m_ui->drawLog();
        ImGui::End();
    }

// --- UI helper definitions --->
#undef _VSPACE
#undef _HSPACE
#undef _SEPARATOR
#undef _PUSHSTYLE_ALPHA
#undef _POPSTYLE
#undef _PUSHDISABLEDIF
#undef _POPDISABLEDIF
#undef _TAG
    // <--- UI helper definitions ---

    // Set UI item active flag
    m_uiState.anyItemActive = ImGui::IsAnyItemActive();

    // Update state from UI back to logic
    m_logic.setState(appState, false);
}
