#pragma once

#include <utility>

template<class T>
class TSingleton
{
public:
    template<typename... Args>
    static T& Instance(Args&&... args)
    {
        if (!s_instance)
        {
            s_instance = new T(std::forward<Args>(args)...);
        }
        return *s_instance;
    }

    static void Destroy()
    {
        if (s_instance != nullptr)
        {
            delete s_instance;
            s_instance = nullptr;
        }
    }

    static bool IsCreated()
    {
        return s_instance != nullptr;
    }

private:
    static T* s_instance;
};

template<class T>
T* TSingleton<T>::s_instance = nullptr;