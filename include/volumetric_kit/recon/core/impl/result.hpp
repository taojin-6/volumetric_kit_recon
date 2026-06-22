// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file core/impl/result.hpp
/// Header-bound implementation of the `Result<T>` template. Included from
/// `core/result.hpp`; not a standalone header.

#include "volumetric_kit/recon/core/check.hpp"

namespace volumetric_kit::recon {

template <class T>
Result<T>::Result(T value) : status_(), value_(std::move(value)) {}

template <class T>
Result<T>::Result(Status err) : status_(std::move(err)) {
  // A Result built from a Status represents a failure; success uses the value
  // constructor. An OK status here is a caller bug, not a runtime error -- fail
  // fast rather than leave ok() true with an empty value_.
  VR_CHECK(!status_.ok(),
           "Result built from an OK Status -- use the value ctor");
}

template <class T>
T& Result<T>::value() & {
  VR_CHECK(ok(), "Result::value() called on an error Result");
  return *value_;
}

template <class T>
const T& Result<T>::value() const& {
  VR_CHECK(ok(), "Result::value() called on an error Result");
  return *value_;
}

template <class T>
T&& Result<T>::value() && {
  VR_CHECK(ok(), "Result::value() called on an error Result");
  return std::move(*value_);
}

template <class T>
T* Result<T>::operator->() {
  return &value();
}

template <class T>
const T* Result<T>::operator->() const {
  return &value();
}

template <class T>
T& Result<T>::operator*() & {
  return value();
}

template <class T>
const T& Result<T>::operator*() const& {
  return value();
}

template <class T>
T&& Result<T>::operator*() && {
  return std::move(value());
}

}  // namespace volumetric_kit::recon
