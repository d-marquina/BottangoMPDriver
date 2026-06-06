# Callbacks that the user can overwrite to add custom logic

def on_effector_registered(identifier, effector_type):
    pass

def on_effector_deregistered(identifier):
    pass

def on_custom_motor_update(identifier, value):
    pass

def on_loop_driven_update(identifier, value):
    pass

def on_curved_event_update(identifier, value):
    pass

def on_onoff_event_changed(identifier, is_on):
    pass

def on_trigger_event_triggered(identifier):
    pass

def on_color_event_update(identifier, r, g, b):
    pass

def on_bottango_connected():
    pass

def on_bottango_deregistered():
    pass
