// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file result.hpp
/// @brief Exception-free, backend-neutral error handling for the public API.
///
/// No exceptions cross the library boundary: mobile (iOS/Android) consumers
/// frequently build with `-fno-exceptions`, where a throwing API is unusable.
/// Fallible calls therefore report failure by value:
///
/// - @ref Status    -- success, or an error domain (@ref Status::Code) with an
///                     optional backend detail code and a context message.
/// - @ref Result    -- a `T` on success, or a `Status` on failure.
///
/// The macro @ref VR_TRY removes the check-and-propagate boilerplate for a
/// `Status` expression, and @ref VR_ASSIGN does the same for a `Result<T>`.
/// Both early-return on failure, so they appear only inside functions that
/// themselves return `Status` or `Result<T>`.
///
/// `Status` is deliberately *backend-neutral*: the detail code is a generic
/// `int64_t` (e.g. a Vulkan `VkResult`, or a `cudaError_t` from the native-CUDA
/// accelerator), never a GPU-API type, so this header -- and the whole `core`
/// tier -- stays free of Vulkan and CUDA includes. Backend tiers wrap their own
/// `..._error` factory and `VR_<backend>_TRY` macros on top.
///
/// Misuse -- reading the value of an error `Result` -- is a programmer error,
/// not a runtime one: it fails fast via `VR_CHECK` (see check.hpp) rather than
/// throwing.
///
/// @code
/// Result<VoxelHashMap> r = VoxelHashMap::create(config);
/// if (!r) return r.status();        // propagate failure to our caller
/// VoxelHashMap& map = r.value();    // safe: guarded by the !r check above
/// @endcode

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "volumetric_kit/recon/core/check.hpp"
#include "volumetric_kit/recon/core/export.hpp"

namespace volumetric_kit::recon {

/// @brief Success, or an error: a domain (@ref Code), an optional backend
/// detail
///        code, and a human-readable message.
///
/// A default-constructed `Status` is success. Build a failure with a domain
/// factory (@ref invalid_argument, @ref not_found, @ref unsupported,
/// @ref out_of_memory, @ref io_error) or, for a failed GPU-backend call,
/// @ref backend_error (which sets @ref domain to @ref Code::Backend and carries
/// the backend's detail code). Convertible to `bool` (true == success) for
/// terse checks.
///
/// @code
/// Status s = integrate(frame);
/// if (!s) {
///   log_message(LogLevel::Error,
///               std::string(to_string(s.domain())) + ": " + s.message());
///   return s;
/// }
/// @endcode
class Status {
 public:
  /// @brief The kind of failure a non-OK `Status` reports.
  ///
  /// This is the primary discriminator. @ref detail carries a meaningful
  /// backend code only when the domain is @ref Code::Backend; for every other
  /// domain it is `0`.
  enum class Code {
    Ok,               ///< Success.
    InvalidArgument,  ///< A malformed or contradictory argument value.
    NotFound,         ///< A named resource or file does not exist.
    Unsupported,      ///< A valid request the device or build cannot satisfy.
    OutOfMemory,      ///< A host or device allocation failed.
    IoError,          ///< A read/write/decode/encode operation failed.
    Backend,          ///< A GPU-backend (Vulkan or the CUDA accelerator) call
                      ///< failed; see @ref detail.
  };

  /// Construct a success status.
  Status() = default;

  /// @brief Build a GPU-backend failure status (domain @ref Code::Backend).
  /// @param detail   Backend-specific error code (e.g. a `VkResult`, or a
  ///                 `cudaError_t`), kept as a generic `int64_t` so `core`
  ///                 stays GPU-API-free.
  /// @param message  Human-readable context (e.g. the failing call site).
  /// @return A non-OK `Status` carrying @p detail and @p message.
  static Status backend_error(std::int64_t detail, std::string message) {
    return Status{Code::Backend, detail, std::move(message)};
  }

  /// @brief Build a non-backend failure status in the named domain.
  /// @param message  Human-readable context.
  /// @return A non-OK `Status` whose @ref domain is the factory's domain and
  ///         whose @ref detail is `0` (no backend call was involved).
  static Status invalid_argument(std::string message) {
    return Status{Code::InvalidArgument, std::move(message)};
  }
  /// @copydoc invalid_argument
  static Status not_found(std::string message) {
    return Status{Code::NotFound, std::move(message)};
  }
  /// @copydoc invalid_argument
  static Status unsupported(std::string message) {
    return Status{Code::Unsupported, std::move(message)};
  }
  /// @copydoc invalid_argument
  static Status out_of_memory(std::string message) {
    return Status{Code::OutOfMemory, std::move(message)};
  }
  /// @copydoc invalid_argument
  static Status io_error(std::string message) {
    return Status{Code::IoError, std::move(message)};
  }

  /// @return `true` if this is a success status.
  bool ok() const noexcept { return domain_ == Code::Ok; }
  /// @return `true` on success (same as @ref ok).
  explicit operator bool() const noexcept { return ok(); }

  /// @return The error domain; @ref Code::Ok exactly when @ref ok.
  Code domain() const noexcept { return domain_; }
  /// @return The backend detail code. Meaningful only when @ref domain is
  ///         @ref Code::Backend; `0` otherwise.
  std::int64_t detail() const noexcept { return detail_; }
  /// @return The failure context message; empty when @ref ok.
  const std::string& message() const noexcept { return message_; }

 private:
  // Non-backend domains carry no detail; this overload fixes detail_ to 0 so a
  // domain factory cannot pair a real code with a non-backend domain. Only
  // backend_error() takes an explicit detail.
  Status(Code domain, std::string message)
      : domain_(domain), message_(std::move(message)) {}
  Status(Code domain, std::int64_t detail, std::string message)
      : domain_(domain), detail_(detail), message_(std::move(message)) {}

  Code domain_ = Code::Ok;
  std::int64_t detail_ = 0;
  std::string message_;
};

/// @brief Human-readable name for a @ref Status::Code (e.g. "InvalidArgument").
/// @param code  A domain value.
/// @return A static, never-empty `string_view`.
VR_CORE_API std::string_view to_string(Status::Code code) noexcept;

/// @brief A value of type `T` on success, or a non-OK @ref Status on failure.
/// @tparam T  The success value type (must be movable).
///
/// Constructs implicitly from either a `T` (success) or a `Status` (failure),
/// so a function can `return value;` or `return some_error;` directly. Always
/// check
/// @ref ok (or the `bool` conversion) before reading @ref value.
///
/// @code
/// Result<Buffer> make_buffer(std::size_t bytes) {
///   if (bytes == 0) return Status::invalid_argument("empty");
///   return Buffer{bytes};   // implicit success
/// }
/// @endcode
template <class T>
class Result {
  // Result<Status> is ill-formed: the implicit constructors from T and from
  // Status would collapse to one signature. A fallible operation that yields no
  // value should return Status directly, not Result<Status>.
  static_assert(!std::is_same_v<T, Status>,
                "Result<Status> is ill-formed; return Status directly for a "
                "valueless fallible operation.");

 public:
  /// Construct a success Result holding @p value.
  Result(T value);  // NOLINT(google-explicit-constructor) -- ergonomic success
                    // return
  /// Construct a failure Result; @p err must be non-OK (checked by VR_CHECK).
  Result(Status err);  // NOLINT(google-explicit-constructor) -- ergonomic error
                       // return

  /// @return `true` if this holds a value rather than an error.
  bool ok() const noexcept { return status_.ok(); }
  /// @return `true` if this holds a value (same as @ref ok).
  explicit operator bool() const noexcept { return ok(); }
  /// @return The status; non-OK exactly when this is an error Result.
  const Status& status() const noexcept { return status_; }

  /// @brief Access the held value.
  /// @pre @ref ok is true. Calling this on an error Result is a programmer
  ///      error: it aborts via `VR_CHECK` (it never throws), so guard with
  ///      @ref ok first.
  /// @return Reference to the held value.
  T& value() &;
  const T& value() const&;
  T&& value() &&;

  /// @brief Member access to the held value (e.g. `result->field`).
  /// @pre @ref ok is true; otherwise aborts, as in @ref value. To read the
  ///      value itself rather than a member, use @ref value.
  T* operator->();
  const T* operator->() const;

 private:
  Status status_;
  std::optional<T> value_;
};

}  // namespace volumetric_kit::recon

/// @brief Evaluate a `Status` expression and early-return it if not OK.
/// @param expr  An expression yielding a `Status`.
///
/// Usable only inside a function returning `Status` or `Result<T>` -- the early
/// `return` carries the failure outward.
///
/// @code
/// Status init() {
///   VR_TRY(allocate());   // returns the error if this fails
///   return {};            // success
/// }
/// @endcode
#define VR_TRY(expr)                                     \
  do {                                                   \
    ::volumetric_kit::recon::Status _vr_status = (expr); \
    if (!_vr_status.ok()) return _vr_status;             \
  } while (0)

/// @brief Evaluate a `Result<T>` expression, early-return its `Status` on
///        failure, otherwise move the value into @p decl.
/// @param decl  A variable declaration (e.g. `VoxelHashMap map`) bound to the
///              unwrapped value on success.
/// @param expr  An expression yielding a `Result<T>`.
///
/// The `Result<T>` analogue of @ref VR_TRY: it removes the check-status-then-
/// move-value boilerplate that fallible-value call sites otherwise repeat.
/// Usable only inside a function returning `Status` or `Result<U>`. Because it
/// declares @p decl in the enclosing scope, it expands to a statement sequence
/// (not a `do { } while`), so it is not a single statement -- never use it as
/// the unbraced body of an `if`/`for`/`while`. The hidden temporary is keyed on
/// `__COUNTER__` (not `__LINE__`), so multiple `VR_ASSIGN`s in one scope never
/// collide. @p decl is a single macro argument, so a type written with a
/// top-level comma needs an alias first.
///
/// @code
/// Result<Mesh> build() {
///   VR_ASSIGN(VoxelHashMap map, VoxelHashMap::create(config));
///   // `map` holds the value here; a failure already returned its Status.
///   return extract(map);
/// }
/// @endcode
#define VR_ASSIGN(decl, expr) VR_ASSIGN_(decl, expr, __COUNTER__)
#define VR_ASSIGN_(decl, expr, id) VR_ASSIGN_IMPL_(decl, expr, id)
#define VR_ASSIGN_IMPL_(decl, expr, id)                       \
  auto _vr_result_##id = (expr);                              \
  if (!_vr_result_##id.ok()) return _vr_result_##id.status(); \
  decl = std::move(_vr_result_##id).value()

#include "volumetric_kit/recon/core/impl/result.hpp"
