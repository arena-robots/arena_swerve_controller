^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package arena_swerve_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add a speed limiter on the commanded body twist: per axis ``linear.x``, ``linear.y`` and ``angular.z`` the parameters ``has_velocity_limits``, ``has_acceleration_limits``, ``min_velocity``, ``max_velocity``, ``min_acceleration`` and ``max_acceleration``. Disabled by default.

0.1.0 (2026-05-12)
------------------
* Initial release. 4-wheel independent steering (swerve) controller for ros2_control.
* Apache-2.0 licensed.
