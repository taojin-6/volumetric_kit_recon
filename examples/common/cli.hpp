// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/cli.hpp
/// @brief The examples' one command-line parser.
///
/// Each flag is declared once, with the variable it sets and the name of its
/// value, and the usage line is made from those declarations, so it cannot
/// leave a flag out. A number is read whole and finite: `10x`, ` 5`, `nan`
/// and an empty value are refused, naming the flag. Header-only, with no
/// recon tier, so every example and its tests include it.
///
/// @code
/// Options opt;
/// vr_example::Cli cli("fuse_replica");
/// cli.positional("scene_dir", opt.scene_dir);
/// cli.option({"-o", "--out"}, "out.ply", opt.out);
/// cli.option("--stride", "N", opt.stride, /*min=*/1);
/// cli.flag("--preload", opt.preload);
/// VKC_TRY(cli.parse(argc, argv));  // the message carries the usage line
/// @endcode

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vr_example {

namespace vkc = volumetric_kit::core;

/// @return Whether @p text starts with white space, which `strtod` and
///         `strtoll` would skip: a number is the whole text.
inline bool starts_with_space(const char* text) {
  return std::isspace(static_cast<unsigned char>(text[0])) != 0;
}

/// @brief Parse @p text, the value of @p flag, as a finite number: the
///        whole text, so `10x`, ` 5` and an empty value are refused.
/// @return OK with @p out set, or InvalidArgument naming @p flag; @p out is
///         unchanged on error.
inline vkc::Status parse_number(const std::string& flag, const char* text,
                                double& out) {
  char* end = nullptr;
  errno = 0;
  const double value = std::strtod(text, &end);
  if (starts_with_space(text) || end == text || *end != '\0' || errno != 0 ||
      !std::isfinite(value)) {
    return vkc::Status::invalid_argument(flag +
                                         ": not a finite number: " + text);
  }
  out = value;
  return {};
}

/// As the double overload, refusing a value past float's range.
inline vkc::Status parse_number(const std::string& flag, const char* text,
                                float& out) {
  double value = 0.0;
  VKC_TRY(parse_number(flag, text, value));
  if (std::abs(value) > std::numeric_limits<float>::max()) {
    return vkc::Status::invalid_argument(flag + ": number too large: " + text);
  }
  out = static_cast<float>(value);
  return {};
}

/// Parse @p text, the value of @p flag, as a decimal int, the whole text.
inline vkc::Status parse_number(const std::string& flag, const char* text,
                                int& out) {
  char* end = nullptr;
  errno = 0;
  const long long value = std::strtoll(text, &end, 10);
  if (starts_with_space(text) || end == text || *end != '\0' || errno != 0 ||
      value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max()) {
    return vkc::Status::invalid_argument(flag + ": not an integer: " + text);
  }
  out = static_cast<int>(value);
  return {};
}

/// Parse @p text, the value of @p flag, as a decimal count: digits only, so
/// a sign is refused as white space is.
inline vkc::Status parse_number(const std::string& flag, const char* text,
                                std::uint32_t& out) {
  char* end = nullptr;
  errno = 0;
  const unsigned long long value = std::strtoull(text, &end, 10);
  if (std::isdigit(static_cast<unsigned char>(text[0])) == 0 || *end != '\0' ||
      errno != 0 || value > std::numeric_limits<std::uint32_t>::max()) {
    return vkc::Status::invalid_argument(flag + ": not a count: " + text);
  }
  out = static_cast<std::uint32_t>(value);
  return {};
}

/// @brief A command line's flags and positional arguments, declared with the
///        variables they set, parsed in one pass.
///
/// A flag that takes a value takes the next argument whatever it looks like,
/// so `--up-axis -y` works; a flag given twice keeps the last. An argument
/// starting with `-` that names no flag is refused, as are a positional
/// argument past those declared, a second flag of a @ref one_of group, and a
/// command line without a positional or a @ref require "required" flag.
/// After the arguments, every @ref check runs, in declaration order. Every
/// error is InvalidArgument, its message `<program>: <what>` followed by the
/// @ref usage line, for a `main` to print as it is.
class Cli {
 public:
  /// A flag's spellings: one (`"--voxel"`) or several (`{"-o", "--out"}`),
  /// the first shown in the usage line.
  struct Names {
    // Implicit, so a flag with one spelling is written as a string.
    Names(const char* name) : list{name} {}
    Names(std::initializer_list<const char*> names)
        : list(names.begin(), names.end()) {}
    std::vector<std::string> list;
  };

  /// What a flag does with its value (null for a switch), given the
  /// spelling it was found under.
  using Handler =
      std::function<vkc::Status(const std::string& flag, const char* value)>;

  /// @param program  The program's name, for the usage line and errors.
  explicit Cli(std::string program) : program_(std::move(program)) {}

  /// @brief A switch: @p out becomes @p value wherever it is given.
  Cli& flag(Names names, bool& out, bool value = true) {
    return on(std::move(names), nullptr,
              [&out, value](const std::string&, const char*) {
                out = value;
                return vkc::Status{};
              });
  }

  /// @brief A flag whose value is parsed as @p out's type: a number by
  ///        @ref parse_number, a `std::string` as given, a
  ///        `std::optional<float>` set from a number.
  /// @param metavar  The value's name in the usage line.
  template <typename T>
  Cli& option(Names names, const char* metavar, T& out) {
    return on(std::move(names), metavar,
              [&out](const std::string& flag, const char* value) {
                return assign(flag, value, out);
              });
  }

  /// @brief An integer flag refusing a value below @p min.
  Cli& option(Names names, const char* metavar, int& out, int min) {
    return on(std::move(names), metavar,
              [&out, min](const std::string& flag, const char* value) {
                int parsed = 0;
                VKC_TRY(parse_number(flag, value, parsed));
                if (parsed < min) {
                  return vkc::Status::invalid_argument(
                      flag + " must be >= " + std::to_string(min));
                }
                out = parsed;
                return vkc::Status{};
              });
  }

  /// @brief As the int overload, into an unsigned field.
  Cli& option(Names names, const char* metavar, std::uint32_t& out,
              std::uint32_t min) {
    return on(std::move(names), metavar,
              [&out, min](const std::string& flag, const char* value) {
                std::uint32_t parsed = 0;
                VKC_TRY(parse_number(flag, value, parsed));
                if (parsed < min) {
                  return vkc::Status::invalid_argument(
                      flag + " must be >= " + std::to_string(min));
                }
                out = parsed;
                return vkc::Status{};
              });
  }

  /// @brief A flag handled by @p handler, given its value -- for a value
  ///        with a grammar of its own (`WxH`, `x,y,z`, a name from a list).
  /// @param metavar  The value's name in the usage line; null for a switch,
  ///                 whose handler is given a null value.
  Cli& on(Names names, const char* metavar, Handler handler) {
    flags_.push_back({std::move(names.list), metavar, std::move(handler),
                      next_group_++, false});
    return *this;
  }

  /// @brief Make the flag declared as @p name one every command line must
  ///        give, or, in a @ref one_of group, the group: the usage line
  ///        shows the flag without brackets, the group in parentheses.
  /// @pre @p name was declared.
  Cli& require(const char* name) {
    declared(name).required = true;
    return *this;
  }

  /// @brief Group flags already declared, of which a command line gives one:
  ///        a second, different one is refused. The usage line shows them
  ///        where the first was declared, `[--serial SN | --rig sync.json]`,
  ///        or `(--up-axis a | --up-vector v)` once @ref require "required".
  /// @pre Each of @p names was declared.
  Cli& one_of(std::initializer_list<const char*> names) {
    VKC_CHECK(names.size() >= 2, "Cli::one_of groups two flags or more");
    const int group = declared(*names.begin()).group;
    for (const char* name : names) join(declared(name).group, group);
    return *this;
  }

  /// @brief Switches that each set @p out to their own value, a @ref one_of
  ///        group: `[--hevc | --mjpeg]`.
  template <typename T>
  Cli& choice(std::vector<std::pair<const char*, T>> switches, T& out) {
    const int group = next_group_;
    for (const std::pair<const char*, T>& option : switches) {
      const T value = option.second;
      on(option.first, nullptr, [&out, value](const std::string&, const char*) {
        out = value;
        return vkc::Status{};
      });
      join(flags_.back().group, group);
    }
    return *this;
  }

  /// @brief A required positional argument, in declaration order.
  Cli& positional(const char* name, std::string& out) {
    positionals_.push_back({name, &out});
    return *this;
  }

  /// @brief A check run after every argument is read: a rule between flags,
  ///        or a default derived from another flag.
  Cli& check(std::function<vkc::Status()> rule) {
    checks_.push_back(std::move(rule));
    return *this;
  }

  /// @brief Read @p argv (its first entry is the program, skipped), then
  ///        run the checks.
  /// @return OK, or InvalidArgument: `<program>: <what>`, then the usage.
  vkc::Status parse(int argc, const char* const* argv) const {
    std::size_t positional = 0;
    // Per group, the flag given first: a group takes one.
    std::vector<const Flag*> given(static_cast<std::size_t>(next_group_));
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      const std::size_t index = index_of(arg);
      if (index == flags_.size()) {
        if (!arg.empty() && arg[0] == '-') {
          return error("unknown flag: " + arg);
        }
        if (positional == positionals_.size()) {
          return error("unexpected argument: " + arg);
        }
        *positionals_[positional++].out = arg;
        continue;
      }
      const Flag& flag = flags_[index];
      const Flag*& first = given[static_cast<std::size_t>(flag.group)];
      if (first != nullptr && first != &flag) {
        return error(names_of(flag.group) + ", not both");
      }
      first = &flag;
      const char* value = nullptr;
      if (flag.metavar != nullptr) {
        if (i + 1 >= argc) return error(arg + " needs a value");
        value = argv[++i];
      }
      if (const vkc::Status s = flag.handler(arg, value); !s.ok()) {
        return error(s.message());
      }
    }
    if (positional < positionals_.size()) {
      return error(std::string("needs <") + positionals_[positional].name +
                   ">");
    }
    for (const Flag& flag : flags_) {
      if (group_required(flag.group) &&
          given[static_cast<std::size_t>(flag.group)] == nullptr) {
        return error("needs " + names_of(flag.group));
      }
    }
    for (const std::function<vkc::Status()>& rule : checks_) {
      if (const vkc::Status s = rule(); !s.ok()) return error(s.message());
    }
    return {};
  }

  /// @return `usage: <program> <positional>... [flag value]...`, wrapped
  ///         under the program's name before 80 columns.
  std::string usage() const {
    const std::string head = "usage: " + program_;
    std::vector<std::string> items;
    for (const Positional& p : positionals_) {
      items.push_back("<" + std::string(p.name) + ">");
    }
    // Each group once, where its first flag was declared.
    std::vector<bool> shown(static_cast<std::size_t>(next_group_), false);
    for (const Flag& flag : flags_) {
      if (shown[static_cast<std::size_t>(flag.group)]) continue;
      shown[static_cast<std::size_t>(flag.group)] = true;
      std::string item;
      bool several = false;
      for (const Flag& member : flags_) {
        if (member.group != flag.group) continue;
        if (!item.empty()) {
          item += " | ";
          several = true;
        }
        item += member.names.front();
        if (member.metavar != nullptr) {
          item += std::string(" ") + member.metavar;
        }
      }
      if (!group_required(flag.group)) {
        item = "[" + item + "]";
      } else if (several) {
        item = "(" + item + ")";
      }
      items.push_back(item);
    }
    constexpr std::size_t kColumns = 80;
    const std::string indent(head.size() + 1, ' ');
    std::string out = head;
    std::size_t line = out.size();
    for (const std::string& item : items) {
      if (line + 1 + item.size() >= kColumns && line > indent.size()) {
        out += "\n" + indent + item;
        line = indent.size() + item.size();
      } else {
        out += " " + item;
        line += 1 + item.size();
      }
    }
    return out;
  }

 private:
  struct Flag {
    std::vector<std::string> names;
    const char* metavar;  // null: a switch
    Handler handler;
    int group;      // a one_of group's flags share one, shown together
    bool required;  // makes its group required
  };
  struct Positional {
    const char* name;
    std::string* out;
  };

  static vkc::Status assign(const std::string&, const char* value,
                            std::string& out) {
    out = value;
    return {};
  }
  template <typename T>
  static vkc::Status assign(const std::string& flag, const char* value,
                            T& out) {
    return parse_number(flag, value, out);
  }
  static vkc::Status assign(const std::string& flag, const char* value,
                            std::optional<float>& out) {
    float parsed = 0.0f;
    VKC_TRY(parse_number(flag, value, parsed));
    out = parsed;
    return {};
  }

  // The index of the flag spelt @p arg; flags_.size() when none is.
  std::size_t index_of(const std::string& arg) const {
    for (std::size_t f = 0; f < flags_.size(); ++f) {
      for (const std::string& name : flags_[f].names) {
        if (name == arg) return f;
      }
    }
    return flags_.size();
  }

  Flag& declared(const char* name) {
    const std::size_t index = index_of(name);
    VKC_CHECK(index < flags_.size(),
              "Cli: a flag named before it was declared");
    return flags_[index];
  }

  // Move every flag of group @p from into group @p to.
  void join(int from, int to) {
    for (Flag& flag : flags_) {
      if (flag.group == from) flag.group = to;
    }
  }

  bool group_required(int group) const {
    for (const Flag& flag : flags_) {
      if (flag.group == group && flag.required) return true;
    }
    return false;
  }

  // "--a or --b": the flags of @p group.
  std::string names_of(int group) const {
    std::string names;
    for (const Flag& flag : flags_) {
      if (flag.group != group) continue;
      names += (names.empty() ? "" : " or ") + flag.names.front();
    }
    return names;
  }

  vkc::Status error(const std::string& what) const {
    return vkc::Status::invalid_argument(program_ + ": " + what + "\n" +
                                         usage());
  }

  std::string program_;
  std::vector<Flag> flags_;
  std::vector<Positional> positionals_;
  std::vector<std::function<vkc::Status()>> checks_;
  int next_group_ = 0;
};

}  // namespace vr_example
