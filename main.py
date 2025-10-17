class City:
    def __init__(self, name:str, connected_cities: dict[object, float]):
        self.name = name
        self.connected_cities = connected_cities
