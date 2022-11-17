import tomli
import robot_utilities

class Config_parser():
    def __init__(self, logger, filename):
        self.logger = logger
        self.filename = filename
        self.data = dict()

    def parse(self):
        f = open(self.filename, "rb")
        try:
            self.data = tomli.load(f)
        except tomli.decoder.TomlDecodeError:
            self.logger.LOGE("Invalid config file!")
        f.close()

    def print_dict(self):
        for key in self.data.keys():
            print(key, self.data[key])

if __name__ == "__main__":
    logger = robot_utilities.Logging(False, True)
    parser = Config_parser(logger, "config/default_config.toml")
    parser.parse()
    parser.print_dict()