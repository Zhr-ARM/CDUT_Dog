"""ROS 参数声明与读取辅助函数。"""

from typing import Any


def declare_parameters(node, defaults: dict[str, Any]) -> None:
    """按默认值批量声明 ROS 参数。

    该函数只负责声明，不负责读取和类型转换。
    """
    for name, default in defaults.items():
        node.declare_parameter(name, default)


def load_parameters(node, defaults: dict[str, Any]) -> dict[str, Any]:
    """批量读取 ROS 参数并按默认值类型进行转换。"""
    return {
        name: _cast_like(node.get_parameter(name).value, default)
        for name, default in defaults.items()
    }


def _cast_like(value: Any, default: Any) -> Any:
    """将参数值转换为与默认值一致的类型。

    说明:
    - 若默认值是列表，则逐项按首元素类型转换。
    - 若默认列表为空，则保持输入可迭代对象的元素原样并转为 list。
    """
    if isinstance(default, list):
        if not default:
            return list(value)
        return [type(default[0])(item) for item in value]
    return type(default)(value)
