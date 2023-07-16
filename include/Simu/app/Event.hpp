////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2023 Matthieu Beauchamp-Boulay
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented;
//    you must not claim that you wrote the original software.
//    If you use this software in a product, an acknowledgment
//    in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such,
//    and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//
////////////////////////////////////////////////////////////

#pragma once

#include "Simu/config.hpp"

namespace simu
{

class Event
{
public:

    enum class Action
    {
        release,
        press,
        repeat // only for Keyboard::Input
    };
};

// redefine glfw's constants as enums to avoid leaking glfw's header into global namespace,
//  keeping it encapsulated in .cpp files.
class Keyboard : public Event
{
public:

    enum Modifier;
    enum class Key;

    struct Input
    {
        static Input fromGlfw(int key, int action, int mods)
        {
            Input input;
            input.action   = static_cast<Action>(action);
            input.modifier = static_cast<Modifier>(mods);
            input.key      = static_cast<Key>(key);
            return input;
        }

        Action   action;
        Modifier modifier;
        Key      key;
    };

    static bool isPressed(Key key);


    enum Modifier
    {
        shift    = (1 << 0),
        control  = (1 << 1),
        alt      = (1 << 2),
        super    = (1 << 3),
        capsLock = (1 << 4),
        numLock  = (1 << 5)
    };

    enum class Key
    {
        unknown = -1,

        space      = ' ',
        apostrophe = '\'',
        comma      = ',',
        minus      = '-',
        period     = '.',
        slash      = '/',

        key0 = '0',
        key1 = '1',
        key2 = '2',
        key3 = '3',
        key4 = '4',
        key5 = '5',
        key6 = '6',
        key7 = '7',
        key8 = '8',
        key9 = '9',

        semicolon = ';',
        equal     = '=',

        A = 'A',
        B = 'B',
        C = 'C',
        D = 'D',
        E = 'E',
        F = 'F',
        G = 'G',
        H = 'H',
        I = 'I',
        J = 'J',
        K = 'K',
        L = 'L',
        M = 'M',
        N = 'N',
        O = 'O',
        P = 'P',
        Q = 'Q',
        R = 'R',
        S = 'S',
        T = 'T',
        U = 'U',
        V = 'V',
        W = 'W',
        X = 'X',
        Y = 'Y',
        Z = 'Z',

        leftBracket  = '[',
        backslash    = '\\',
        rightBracket = ']',
        graveAccent  = '`',

        world1 = 161,
        world2 = 162,

        escape = 256,
        enter,
        tab,
        backspace,
        insert,
        del,

        right,
        left,
        down,
        up,

        pageUp,
        pageDown,
        home,
        end,

        capsLock = 280,
        scrollLock,
        numLock,

        printScreen,
        pause,

        F1 = 290,
        F2,
        F3,
        F4,
        F5,
        F6,
        F7,
        F8,
        F9,
        F10,
        F11,
        F12,
        F13,
        F14,
        F15,
        F16,
        F17,
        F18,
        F19,
        F20,
        F21,
        F22,
        F23,
        F24,
        F25,

        keypad0 = 320,
        keypad1,
        keypad2,
        keypad3,
        keypad4,
        keypad5,
        keypad6,
        keypad7,
        keypad8,
        keypad9,

        keypadDecimal,
        keypadDivide,
        keypadMultiply,
        keypadSubstract,
        keypadAdd,
        keypadEnter,
        keypadEqual,

        leftShift = 340,
        leftControl,
        leftAlt,
        leftSuper,

        rightShift,
        rightControl,
        rightAlt,
        rightSuper,

        menu
    };
};


class Mouse : public Event
{
public:

    enum class Button
    {
        button1 = 0,
        button2,
        button3,
        button4,
        button5,
        button6,
        button7,
        button8,

        left   = button1,
        right  = button2,
        middle = button3
    };

    struct Input
    {
        static Input fromGlfw(Vec2 pos, int button, int action, int mods)
        {
            Input input{pos};
            input.button    = static_cast<Button>(button);
            input.action    = static_cast<Action>(action);
            input.modifiers = static_cast<Keyboard::Modifier>(mods);
            return input;
        }

        Vec2 pos; // given in scene coordinates

        Button             button;
        Action             action;
        Keyboard::Modifier modifiers;
    };
};

} // namespace simu
