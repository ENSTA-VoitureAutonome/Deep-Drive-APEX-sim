"""Third-party-style copies of simulation modules the driving logic imports.

``core/sim_paths.ensure_tools_path()`` puts this directory on ``sys.path`` so
``import pose_dataset`` resolves here. Only the objects the car side is allowed
to use are copied; see each module's docstring.
"""
