Interactive Auton Selector
===========================

Our user interface utilizes **LVGL v9** to present a visual representation of the game field alongside details on autonomous choices.

Features
--------

* **Automated Scroll-Height Detection**: Built-in container metrics calculate the height of description labels at runtime. If description lines are cut off by the fixed bottom **CONFIRM SELECTION** button, scrolling is automatically activated.
* **Scroll-Chaining**: If content overflows, touch-drag inputs anywhere on the screen propagate smoothly.
