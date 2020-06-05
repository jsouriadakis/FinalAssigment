import enum


class NodeTypeEnum(enum.Enum):

    TRANSFORM_OUT = "IGTL_TRANSFORM_OUT"

    IMPORTER = "igtl_importer"

    POINT_IN = "IGTL_POINT_IN"

    MOVE_GROUP_PLANNED_PATH = "/move_group/display_planned_path"
