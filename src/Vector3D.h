#pragma once

#include <string>

template <typename T>
struct Vector3D {
public:
    T x, y, z;

    static constexpr auto splat(T v) {
        return Vector3D<T>{v, v, v};
    }

    static constexpr auto zero() {
        return splat(0);
    }

    /// Return vector {0, 1, 2}
    static constexpr auto indices() {
        return Vector3D<T>{0, 1, 2};
    }

    /*
    Abs is not defined because we don't want to rely on a square root calculation
    constexpr auto abs() const {
        return sqrt(absSquared());
    }
    */

    constexpr auto absSquared() const {
        return (*this) * (*this);
    }

    constexpr auto operator-() const {
        return apply([](auto v) { return -v; });
    }

    /// Create a new vector by applying a function to each element.
    template <typename F>
    constexpr auto apply(const F& f) const {
        return Vector3D<decltype(f(x))>{f(x), f(y), f(z)};
    }

    /// Combine two vectors by applying a binary function to each pair of elements
    template <typename U, typename F>
    constexpr auto apply2(const Vector3D<U>& other, const F& f) const {
        return Vector3D<decltype(f(x, other.x))>(
            f(x, other.x), f(y, other.y), f(z, other.z)
        );
    }

    /// Reduce the vector to a single value by applying an associative binary function
    template <typename F>
    constexpr auto reduce(const F& f) const {
        return f(f(x, y), z);
    }

    std::string str() const {
        return apply([](auto v) { return std::to_string(v); }).
            reduce([](auto a, auto b) { return a.append(", ").append(b); });
    }
};

template <typename T, typename U>
constexpr auto operator+(const Vector3D<T>& a, const Vector3D<U>& b) {
    return a.apply2(b, [](auto va, auto vb) { return va + vb; });
}

template <typename T, typename U>
constexpr auto operator-(const Vector3D<T>& a, const Vector3D<U>& b) {
    return a.apply2(b, [](auto va, auto vb) { return va - vb; });
}

/// Dot product
template <typename T, typename U>
constexpr auto operator*(const Vector3D<T>& a, const Vector3D<U>& b) {
    return a.apply2(
        b,
        [](auto va, auto vb) { return va * vb; }
    ).reduce(
        [](auto va, auto vb) { return va + vb; }
    );
}

template <typename T, typename U>
constexpr auto operator*(T a, const Vector3D<U>& b) {
    return b.apply([&](auto v) { return a * v; });
}

template <typename T, typename U>
constexpr auto operator*(const Vector3D<T>& a, U b) {
    return a.apply([&](auto v) { return v * b; });
}

template <typename T, typename U>
constexpr auto operator/(const Vector3D<T>& a, U b) {
    return a.apply([&](auto v) { return v / b; });
}

template <typename T, typename U>
constexpr auto operator==(const Vector3D<T>& a, const Vector3D<U>& b) {
    return a.apply2(
        b,
        [](auto va, auto vb) { return va == vb; }
    ).reduce(
        [](auto va, auto vb) { return va && vb; }
    );
}

