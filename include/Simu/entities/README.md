### Purpose

The entities folder defines the necessary classes for an entity-component architecture.
This is not a true ECS as systems are not explicitly defined.
Instead, systems are defined by the usage from the engine.
This is because order of execution is important, it is pointless to have a system scheduler try to optimize system
executions.

The main goal of using the component system is to improve performance by increasing cache locality.
It also favors using composition over inheritance, making it easier to have dynamic behavior without virtual functions
which require objects to be allocated separately.

### Usage

The `Entities` class is templated on the components it can store.
This simplifies the storage of components and is not restrictive since the entities is internal to the engine and all
components are known at compile time.

```c++
#include "Simu/entities/Entities.hpp"

using namespace simu;

struct Position {
    float x, y;
};

struct Velocity {
    float x, y;
};

int main() {
    Entities<Position, Velocity> entities;
    
    Entity e = entities.create();
    entities.add<Position>(e, {1, 2});
    entities.add<Velocity>(e, {3, 4});
    
    entities.query<Position, Velocity>([](Position& pos, Velocity& vel) {
        pos.x += vel.x;
        pos.y += vel.y;
    });
    
    entities.destroy(e);
}
```



