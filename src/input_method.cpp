#include "aerial_touch/input_method.hpp"

#include <array>
#include <cstring>

namespace aerial_touch {
namespace {

// Indexed by digit. Key 0 types a space in letter mode, so it has no group of its own.
constexpr std::array<const char*, 10> kLetterGroups{ "", ".,?!", "abc", "def", "ghi",
                                                     "jkl", "mno", "pqrs", "tuv", "wxyz" };
constexpr std::array<char, 10> kSymbols{ '/', '.', ',', '?', '!', '@', '#', '+', '-', '*' };

std::optional<int> digit_of(const std::string& key) {
    if(key.size() != 1U || key[0] < '0' || key[0] > '9') {
        return std::nullopt;
    }
    return key[0] - '0';
}

std::size_t group_size(const int digit) {
    return std::strlen(kLetterGroups[static_cast<std::size_t>(digit)]);
}

}  // namespace

InputMethod::InputMethod(const InputMethodConfig config) : config_(config) {}

void InputMethod::press(const std::string& key, const std::int64_t timestamp_ms) {
    if(key == kModeKey) {
        if(!candidates_.empty()) {
            // During selection the mode key backs out of it. Switching mode here as well would
            // leave the operator in a different mode with nothing typed, which is two surprises.
            candidates_.clear();
            return;
        }
        commit_pending();
        mode_ = next_input_mode(mode_);
        return;
    }
    if(key == kDeleteKey) {
        if(!candidates_.empty()) {
            candidates_.clear();
        }
        else if(pending_.has_value()) {
            pending_.reset();
        }
        else if(!text_.empty()) {
            text_.pop_back();
        }
        return;
    }

    const auto digit = digit_of(key);
    if(!digit.has_value()) {
        return;
    }
    if(!candidates_.empty()) {
        // Only the lit keys pick a letter. Any other key is most likely a mis-press beside them, so
        // it is ignored rather than allowed to start a different letter group.
        if(*digit >= 1 && static_cast<std::size_t>(*digit) <= candidates_.size()) {
            text_ += candidates_[static_cast<std::size_t>(*digit) - 1U];
            candidates_.clear();
        }
        return;
    }

    switch(mode_) {
    case InputMode::Number:
        text_ += static_cast<char>('0' + *digit);
        break;
    case InputMode::Symbol:
        text_ += kSymbols[static_cast<std::size_t>(*digit)];
        break;
    case InputMode::Letter:
        press_letter_key(*digit, timestamp_ms);
        break;
    }
}

void InputMethod::press_letter_key(const int digit, const std::int64_t timestamp_ms) {
    if(digit == 0) {
        commit_pending();
        text_ += ' ';
        return;
    }
    if(config_.letter_selection == LetterSelection::TwoStep) {
        const char* group = kLetterGroups[static_cast<std::size_t>(digit)];
        candidates_.assign(group, group + std::strlen(group));
        return;
    }
    if(pending_.has_value() && pending_->digit == digit
       && timestamp_ms - pending_->last_press_ms < config_.multitap_timeout_ms) {
        pending_->index = (pending_->index + 1U) % group_size(digit);
        pending_->last_press_ms = timestamp_ms;
        return;
    }
    // A different key, or the same key after the pause, settles the previous letter first. That is
    // what lets "ll" be typed: press 5 three times, wait, press it three times again.
    commit_pending();
    pending_ = PendingLetter{ digit, 0U, timestamp_ms };
}

void InputMethod::tick(const std::int64_t timestamp_ms) {
    if(pending_.has_value() && timestamp_ms - pending_->last_press_ms >= config_.multitap_timeout_ms) {
        commit_pending();
    }
}

void InputMethod::commit_pending() {
    if(pending_.has_value()) {
        text_ += kLetterGroups[static_cast<std::size_t>(pending_->digit)][pending_->index];
        pending_.reset();
    }
}

void InputMethod::reset() {
    mode_ = InputMode::Number;
    text_.clear();
    candidates_.clear();
    pending_.reset();
}

InputMode InputMethod::mode() const {
    return mode_;
}

const std::string& InputMethod::text() const {
    return text_;
}

std::optional<char> InputMethod::pending_letter() const {
    if(!pending_.has_value()) {
        return std::nullopt;
    }
    return kLetterGroups[static_cast<std::size_t>(pending_->digit)][pending_->index];
}

const std::vector<char>& InputMethod::candidates() const {
    return candidates_;
}

KeyLabel InputMethod::label(const std::string& key) const {
    if(key == kModeKey) {
        return { candidates_.empty() ? input_mode_name(next_input_mode(mode_)) : "ESC", "" };
    }
    if(key == kDeleteKey) {
        return { "DEL", "" };
    }
    const auto digit = digit_of(key);
    if(!digit.has_value()) {
        return { key, "" };
    }
    const std::string digit_text(1, key[0]);
    if(!candidates_.empty()) {
        if(*digit >= 1 && static_cast<std::size_t>(*digit) <= candidates_.size()) {
            return { std::string(1, candidates_[static_cast<std::size_t>(*digit) - 1U]), "", true, false };
        }
        return { digit_text, "", false, true };
    }
    switch(mode_) {
    case InputMode::Number:
        return { digit_text, "" };
    case InputMode::Letter:
        return { digit_text, *digit == 0 ? "spc" : kLetterGroups[static_cast<std::size_t>(*digit)] };
    case InputMode::Symbol:
        return { std::string(1, kSymbols[static_cast<std::size_t>(*digit)]), digit_text };
    }
    return { digit_text, "" };
}

const char* input_mode_name(const InputMode mode) {
    switch(mode) {
    case InputMode::Number:
        return "123";
    case InputMode::Letter:
        return "ABC";
    case InputMode::Symbol:
        return "#+=";
    }
    return "123";
}

InputMode next_input_mode(const InputMode mode) {
    switch(mode) {
    case InputMode::Number:
        return InputMode::Letter;
    case InputMode::Letter:
        return InputMode::Symbol;
    case InputMode::Symbol:
        return InputMode::Number;
    }
    return InputMode::Number;
}

const char* letter_selection_name(const LetterSelection selection) {
    return selection == LetterSelection::MultiTap ? "multitap" : "two_step";
}

std::optional<LetterSelection> parse_letter_selection(const std::string& text) {
    if(text == "two_step") {
        return LetterSelection::TwoStep;
    }
    if(text == "multitap") {
        return LetterSelection::MultiTap;
    }
    return std::nullopt;
}

}  // namespace aerial_touch
