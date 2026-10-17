#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace aerial_touch {

// The two cells beside "0" in the fourth row, which used to be dead space. Like the * and # keys of
// a phone keypad they carry commands rather than characters.
inline constexpr const char* kModeKey = "mode";
inline constexpr const char* kDeleteKey = "del";

enum class InputMode {
    Number,
    Letter,
    Symbol,
};

enum class LetterSelection {
    // Press a letter key and its letters move onto keys 1-4; a second press picks one. Always two
    // presses per letter and no timer, which suits mid-air pressing: every press costs a retreat
    // past the release threshold, and a missed press is visible instead of silently changing the
    // letter.
    TwoStep,
    // Phone style: pressing the same key again cycles through its letters, and a pause commits.
    MultiTap,
};

struct InputMethodConfig {
    LetterSelection letter_selection{ LetterSelection::TwoStep };
    std::int64_t multitap_timeout_ms{ 1200 };
};

// What one keypad cell shows in the current state. ASCII only, because the labels are drawn with
// OpenCV's Hershey fonts.
struct KeyLabel {
    std::string main;
    std::string sub;
    bool candidate{ false };  // offers a letter during two-step selection
    bool disabled{ false };   // does nothing in the current state
};

class InputMethod {
public:
    explicit InputMethod(InputMethodConfig config = {});

    // `key` is a keypad cell: "0"-"9", kModeKey or kDeleteKey. Unknown cells are ignored.
    void press(const std::string& key, std::int64_t timestamp_ms);
    // Commits a multi-tap letter once the pause since its last press reaches the timeout.
    void tick(std::int64_t timestamp_ms);
    void reset();

    InputMode mode() const;
    const std::string& text() const;
    // The multi-tap letter still cycling, not yet part of text().
    std::optional<char> pending_letter() const;
    // Letters waiting for a pick in two-step selection; empty when not selecting.
    const std::vector<char>& candidates() const;
    KeyLabel label(const std::string& key) const;

private:
    struct PendingLetter {
        int digit{};
        std::size_t index{};
        std::int64_t last_press_ms{};
    };

    void press_letter_key(int digit, std::int64_t timestamp_ms);
    void commit_pending();

    InputMethodConfig config_;
    InputMode mode_{ InputMode::Number };
    std::string text_;
    std::vector<char> candidates_;
    std::optional<PendingLetter> pending_;
};

// Short name shown on the mode key and in the HUD: "123", "ABC" or "#+=".
const char* input_mode_name(InputMode mode);
InputMode next_input_mode(InputMode mode);
// YAML spelling: "two_step" or "multitap".
const char* letter_selection_name(LetterSelection selection);
std::optional<LetterSelection> parse_letter_selection(const std::string& text);

}  // namespace aerial_touch
