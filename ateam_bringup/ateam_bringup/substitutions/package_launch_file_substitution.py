from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


class PackageLaunchFileSubstitution(PathJoinSubstitution):
    """Substitution that builds a path given a package and launch file name"""

    def __init__(self, package_name: SomeSubstitutionsType, file_name: SomeSubstitutionsType) -> None:
        super().__init__([get_package_share_directory(package_name), "launch", file_name])
