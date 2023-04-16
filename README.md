# KRCC
<iframe src="//player.bilibili.com/player.html?aid=269310280&bvid=BV1Qc411V7mT&cid=1077292493&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>

## Introduction
Kinematic rigidbody character controller is a character controller system based on Unity's internal physics engine. It is designed to be easy to use and easy to extend. With the component correctly configured, 
you can use it to move the character in the scene using APIs similar to Unity's built-in character controller Component.

In order to make it familiar to users, we not only make the APIs familiar, but also make the icons more like native icons.

### Features
- Using Collide and Slide to move the character, which make the character more stable than rigidbody character controller
- Support for moving in 3D space (instead of the non-rotation compatible internal character controller)
- Support for dynamic moving platforms (With up-axis lockable to make interesting dynamic platforms)
- Support for spherical gravity (Component in development, but you can certainly make your own)

## How to use
There is overview of how to use the component. May not be that detailed.
But Considering that the component is very easy to use and the source code I offered is not complicated, you can also refer to the example project.
### 1. Download and import the package
Download the package and import it into your project.
You could also use git submodule.

### 2. Add the KRCC component to the character
Add the KRCC component to the character you want to control.
You can find the component under Physics/KRCC/Kinematic Rigidbody CC in the component menu.

### 3. Configure the component
You can easily note that the component need two other components to run.
- Capsule Collider
- Rigidbody (with _isKinematic_ checked, _useGravity_ unchecked)

#### 3.1 Rigidbody
The Rigidbody component is used to control the movement of the character.
We need to keep _isKinematic_ checked to tell the engine that we handles the collision manually.

#### 3.2 Capsule Collider
The origin is designed to be at the bottom of the character. So do make sure
that the character model and capsule collider are at desired position.

### 4. Use the API to control the character
Write your own script to make the character move.
```c#
// Get the KRCC component
private KinematicRigidbodyCharacterController _krcc;
/*
...
*/
private void FixedUpdate()
{
    // Get the input
    float horizontal = Input.GetAxis("Horizontal");
    float vertical = Input.GetAxis("Vertical");
    // Move the character
    _krcc.Move(new Vector3(horizontal, 0, vertical));
}
```
Make sure you call the Move method in FixedUpdate. We also provide a _MoveRotation()_ method to rotate the character.

The component does not make the character move or rotate by it self.
You need to call the Move method to make the character move, including when you are trying to apply gravity to the character when falling.
This is designed to provide you the greatest control over the movement.

You can get and calculate the velocity of the character and apply it to the character using property.
```c#
// Get the KRCC Base Velocity， which means local velocity to the dynamic platform
_krcc.BaseVelocity;
```

### (Additional) 5. Configure the Dynamic Platform
If you want to make a dynamic platform, you can add a Rigidbody component to the platform and make sure _isKinematic_ checked.
Then you can add a KRCC component to the platform and configure it as a dynamic platform.

There is only one option which is _Is Sync Up Axis_. If you uncheck it, the platform will always keep the up axis of the character unchanged, 
but rotations around other axis will be applied.

## How to extend
For more details about the algorithm, you can read my blog post: [Kinematic Rigidbody Character Controller](https://zhuanlan.zhihu.com/p/610948152).

If you want to make your own spherical gravity component, you can change the gravity property and make move the character as you wish.
