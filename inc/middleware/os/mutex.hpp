// In a file like embedded_compat.hpp
#pragma once

namespace OS {
    // Simple mutex that doesn't actually lock anything
    class mutex {
    public:
        mutex() = default;
        ~mutex() = default;
        void lock() {}
        void unlock() {}
        bool try_lock() { return true; }
    };

    template<typename T>
    class lock_guard {
    public:
        explicit lock_guard(T& m) {}
        ~lock_guard() {}
    };

    // Simple function implementation
    template<typename Signature>
    class function;

    template<typename Ret, typename... Args>
    class function<Ret(Args...)> {
    private:
        typedef Ret (*FunctionPointer)(Args...);
        FunctionPointer ptr;

    public:
        function() : ptr(nullptr) {}
        function(FunctionPointer p) : ptr(p) {}

        Ret operator()(Args... args) const {
            if (ptr) return ptr(args...);
            return Ret();
        }

        operator bool() const { return ptr != nullptr; }
    };
}