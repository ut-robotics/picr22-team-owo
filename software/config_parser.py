import tomli
import robot_utilities

class Config_parser():
    def __init__(self, filename):
        self.filename = filename
        self.data = dict()

    def parse(self):
        f = open("config/" + self.filename, "rb")
        try:
            self.data = tomli.load(f)
        except tomli.decoder.TomlDecodeError:
            print("ERROR: Invalid config filename:", self.filename)
        f.close()
        print(self.data)

    def print_config(self):
        print("Title:", self.data["title"])
        print(self.data["logic"])

        print("\nModules")
        print("Referee", self.data["modules"]["referee"])
        print("Logger", self.data["modules"]["logger"])
        print("Manual", self.data["modules"]["manual"])
        print("Mainboard", self.data["modules"]["mainboard"])
        print("Camera", self.data["modules"]["camera"])

        print("\nStates")
        print("start_go", self.data["states"]["start_go"])
        print("ball_search", self.data["states"]["ball_search"])
        print("ball_move", self.data["states"]["ball_move"])
        print("ball_orbit", self.data["states"]["ball_orbit"])
        print("ball_throw", self.data["states"]["ball_throw"])
        print("manual", self.data["states"]["manual"])

    def get_logic_dict(self):
        return self.data["logic"]

    def get_module_dict(self, module_key):
        try:
            return self.data["modules"][module_key]
        except KeyError:
            print("ERROR: Invalid module name:", module_key)

    def get_state_dict(self, state_key):
        try:
            return self.data["states"][state_key]
        except KeyError:
            print("ERROR: Invalid state name:", state_key)



if __name__ == "__main__":
    parser = Config_parser("config/default_config.toml")
    parser.parse()
    parser.print_config()
    print()
    print(parser.get_state_dict("ball_orbit"))
    print(parser.get_module_dict("referee"))