Welcome to Helix API
===================

Helix is a robust, developer-first autonomous framework for VEX V5 robots running PROS 4. This documentation provides complete technical specifics on our coordinate tracking, motion profiles, custom PID modules, and dynamic on-brain user interfaces.

.. toctree::
   :maxdepth: 2
   :caption: Helix Core Architecture:

   chassis_api
   pid_api
   selectors_api

.. note::
   The Helix API utilizes a modular, object-oriented design. This admonition box is demonstrates our seamless glassmorphic styling, ensuring crucial technical updates fit the UI aesthetic.

Doxygen Integration
-------------------

Helix provides comprehensive Doxygen-compatible docstrings within the C++ source files. This allows automatic integration of inline code definitions and type signatures directly into this documentation site.

See our :ref:`pid_api` section for a live example of dynamically generated feedback control specifics.
