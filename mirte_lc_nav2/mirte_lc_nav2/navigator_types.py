'''
    Navigator types
'''

class SystematicNavigator():
    def __init__(self, resolution=0.1):
        self.navigator_type = 'systematic'
        self.path = None
        self.map = None
        self.resolution = resolution
    
    def update_map(self, new_map):
        self.map = new_map
        self.generate_path()
        return self.path
    
    def generate_path(self):
        pass

class ReactiveNavigator():
    def __init__(self):
        self.navigator_type = 'reactive'