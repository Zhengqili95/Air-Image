#include "aerial_touch/input_method.hpp"
#include "aerial_touch/keypad.hpp"

#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

using aerial_touch::InputMethod;
using aerial_touch::InputMode;
using aerial_touch::LetterSelection;

InputMethod letter_input(const LetterSelection selection) {
    InputMethod input({ selection, 1200 });
    input.press(aerial_touch::kModeKey, 0);
    return input;
}

void press_all(InputMethod& input, std::initializer_list<const char*> keys, std::int64_t& timestamp_ms) {
    for(const char* key : keys) {
        input.press(key, timestamp_ms);
        timestamp_ms += 300;
    }
}

bool number_mode_types_digits() {
    InputMethod input;
    std::int64_t now = 0;
    press_all(input, { "1", "2", "0", "9" }, now);
    return input.mode() == InputMode::Number && input.text() == "1209";
}

bool mode_key_cycles_number_letter_symbol() {
    InputMethod input;
    const bool starts_in_numbers = input.mode() == InputMode::Number;
    input.press(aerial_touch::kModeKey, 0);
    const bool letters = input.mode() == InputMode::Letter;
    input.press(aerial_touch::kModeKey, 10);
    const bool symbols = input.mode() == InputMode::Symbol;
    input.press(aerial_touch::kModeKey, 20);
    return starts_in_numbers && letters && symbols && input.mode() == InputMode::Number && input.text().empty();
}

bool symbol_mode_uses_the_symbol_table() {
    InputMethod input;
    input.press(aerial_touch::kModeKey, 0);
    input.press(aerial_touch::kModeKey, 10);
    std::int64_t now = 20;
    press_all(input, { "1", "5", "7", "0" }, now);
    return input.text() == ".@+/";
}

bool two_step_offers_the_group_and_types_the_pick() {
    auto input = letter_input(LetterSelection::TwoStep);
    input.press("7", 100);
    const bool offers_pqrs = input.candidates() == std::vector<char>({ 'p', 'q', 'r', 's' }) && input.text().empty();
    input.press("4", 200);
    return offers_pqrs && input.text() == "s" && input.candidates().empty();
}

// A key that is not lit during selection is most likely a mis-press beside the lit ones; starting a
// different letter from it would turn one slip into a wrong character.
bool two_step_ignores_keys_that_are_not_candidates() {
    auto input = letter_input(LetterSelection::TwoStep);
    input.press("2", 100);
    input.press("9", 200);
    const bool still_selecting = input.candidates().size() == 3U && input.text().empty();
    input.press("2", 300);
    return still_selecting && input.text() == "b";
}

bool two_step_mode_and_delete_back_out_of_selection() {
    auto input = letter_input(LetterSelection::TwoStep);
    input.press("3", 100);
    input.press(aerial_touch::kModeKey, 200);
    const bool mode_cancelled = input.candidates().empty() && input.mode() == InputMode::Letter;
    input.press("3", 300);
    input.press(aerial_touch::kDeleteKey, 400);
    return mode_cancelled && input.candidates().empty() && input.text().empty();
}

bool multitap_cycles_and_commits_after_the_pause() {
    auto input = letter_input(LetterSelection::MultiTap);
    std::int64_t now = 100;
    press_all(input, { "7", "7", "7" }, now);  // last press at 700
    const bool pending_r = input.pending_letter() == 'r' && input.text().empty();
    input.tick(1899);
    const bool still_open = input.pending_letter().has_value();
    input.tick(1900);
    return pending_r && still_open && input.text() == "r" && !input.pending_letter().has_value();
}

bool multitap_another_key_commits_the_previous_letter() {
    auto input = letter_input(LetterSelection::MultiTap);
    input.press("2", 100);
    input.press("3", 200);
    return input.text() == "a" && input.pending_letter() == 'd';
}

// "hello" needs two l's from the same key: after the pause the next press must start a new letter,
// even when that press arrives before any tick() has committed the first one.
bool multitap_types_the_same_key_twice_after_a_pause() {
    auto input = letter_input(LetterSelection::MultiTap);
    std::int64_t now = 100;
    press_all(input, { "5", "5", "5" }, now);  // l, last press at 700
    now = 2100;
    press_all(input, { "5", "5", "5" }, now);  // l again
    input.tick(5000);
    return input.text() == "ll";
}

bool multitap_wraps_around_the_group() {
    auto input = letter_input(LetterSelection::MultiTap);
    std::int64_t now = 100;
    press_all(input, { "2", "2", "2", "2" }, now);
    return input.pending_letter() == 'a';
}

bool delete_discards_the_open_letter_before_erasing_text() {
    auto input = letter_input(LetterSelection::MultiTap);
    input.press("2", 100);
    input.press("3", 200);  // "a" committed, "d" open
    input.press(aerial_touch::kDeleteKey, 300);
    const bool discarded = input.text() == "a" && !input.pending_letter().has_value();
    input.press(aerial_touch::kDeleteKey, 400);
    input.press(aerial_touch::kDeleteKey, 500);  // deleting from empty text is harmless
    return discarded && input.text().empty();
}

bool zero_types_a_space_after_the_open_letter() {
    auto input = letter_input(LetterSelection::MultiTap);
    input.press("4", 100);
    input.press("0", 200);
    return input.text() == "g ";
}

bool switching_mode_commits_the_open_letter() {
    auto input = letter_input(LetterSelection::MultiTap);
    input.press("6", 100);
    input.press(aerial_touch::kModeKey, 200);
    return input.text() == "m" && input.mode() == InputMode::Symbol;
}

bool labels_follow_the_mode_and_the_selection() {
    InputMethod input;
    const bool numbers = input.label("2").main == "2" && input.label("2").sub.empty()
                         && input.label(aerial_touch::kModeKey).main == "ABC"
                         && input.label(aerial_touch::kDeleteKey).main == "DEL";
    input.press(aerial_touch::kModeKey, 0);
    const bool letters = input.label("7").main == "7" && input.label("7").sub == "pqrs"
                         && input.label("0").sub == "spc" && input.label(aerial_touch::kModeKey).main == "#+=";
    input.press("7", 100);
    const auto pick = input.label("4");
    const auto idle = input.label("5");
    const bool selecting = pick.main == "s" && pick.candidate && idle.disabled
                           && input.label(aerial_touch::kModeKey).main == "ESC";
    return numbers && letters && selecting;
}

bool reset_returns_to_empty_number_mode() {
    auto input = letter_input(LetterSelection::TwoStep);
    input.press("2", 100);
    input.reset();
    return input.mode() == InputMode::Number && input.text().empty() && input.candidates().empty();
}

bool letter_selection_names_round_trip() {
    using aerial_touch::letter_selection_name;
    using aerial_touch::parse_letter_selection;
    return parse_letter_selection(letter_selection_name(LetterSelection::TwoStep)) == LetterSelection::TwoStep
           && parse_letter_selection(letter_selection_name(LetterSelection::MultiTap)) == LetterSelection::MultiTap
           && !parse_letter_selection("t9").has_value();
}

// The fourth-row corners used to be dead space beside "0"; they now carry the input commands.
bool keypad_fourth_row_corners_are_mode_and_delete() {
    const aerial_touch::Keypad keypad({ 240.0F, 320.0F, 80.0F, 80.0F, 0.0F, 0.0F });
    return keypad.key_at({ 40.0F, 280.0F }).value_or("?") == aerial_touch::kModeKey
           && keypad.key_at({ 120.0F, 280.0F }).value_or("?") == "0"
           && keypad.key_at({ 200.0F, 280.0F }).value_or("?") == aerial_touch::kDeleteKey
           && keypad.regions().size() == 12U;
}

}  // namespace

bool run_input_method_tests() {
    const std::initializer_list<std::pair<const char*, bool (*)()>> tests{
        { "number_mode_types_digits", &number_mode_types_digits },
        { "mode_key_cycles_number_letter_symbol", &mode_key_cycles_number_letter_symbol },
        { "symbol_mode_uses_the_symbol_table", &symbol_mode_uses_the_symbol_table },
        { "two_step_offers_the_group_and_types_the_pick", &two_step_offers_the_group_and_types_the_pick },
        { "two_step_ignores_keys_that_are_not_candidates", &two_step_ignores_keys_that_are_not_candidates },
        { "two_step_mode_and_delete_back_out_of_selection", &two_step_mode_and_delete_back_out_of_selection },
        { "multitap_cycles_and_commits_after_the_pause", &multitap_cycles_and_commits_after_the_pause },
        { "multitap_another_key_commits_the_previous_letter", &multitap_another_key_commits_the_previous_letter },
        { "multitap_types_the_same_key_twice_after_a_pause", &multitap_types_the_same_key_twice_after_a_pause },
        { "multitap_wraps_around_the_group", &multitap_wraps_around_the_group },
        { "delete_discards_the_open_letter_before_erasing_text", &delete_discards_the_open_letter_before_erasing_text },
        { "zero_types_a_space_after_the_open_letter", &zero_types_a_space_after_the_open_letter },
        { "switching_mode_commits_the_open_letter", &switching_mode_commits_the_open_letter },
        { "labels_follow_the_mode_and_the_selection", &labels_follow_the_mode_and_the_selection },
        { "reset_returns_to_empty_number_mode", &reset_returns_to_empty_number_mode },
        { "letter_selection_names_round_trip", &letter_selection_names_round_trip },
        { "keypad_fourth_row_corners_are_mode_and_delete", &keypad_fourth_row_corners_are_mode_and_delete },
    };

    bool all_passed = true;
    for(const auto& test : tests) {
        if(!test.second()) {
            std::cerr << "FAILED: " << test.first << "\n";
            all_passed = false;
        }
    }
    return all_passed;
}
