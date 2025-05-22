from sensor_msgs.msg import PointCloud2,  PointField  # 导入点云和点云字段消息类型

import sys  # 导入系统模块
import math  # 导入数学模块
import struct  # 导入结构体模块

_DATATYPES = {}  # 定义数据类型字典
_DATATYPES[PointField.INT8] = ('b', 1)  # 8位有符号整型
_DATATYPES[PointField.UINT8] = ('B', 1)  # 8位无符号整型
_DATATYPES[PointField.INT16] = ('h', 2)  # 16位有符号整型
_DATATYPES[PointField.UINT16] = ('H', 2)  # 16位无符号整型
_DATATYPES[PointField.INT32] = ('i', 4)  # 32位有符号整型
_DATATYPES[PointField.UINT32] = ('I', 4)  # 32位无符号整型
_DATATYPES[PointField.FLOAT32] = ('f', 4)  # 32位浮点型
_DATATYPES[PointField.FLOAT64] = ('d', 8)  # 64位浮点型


def read_points(cloud, field_names=None, skip_nans=False, uvs=[], x_range=None, y_range=None, z_range=None):  # 读取点云数据的函数
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(
        cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'  # 确保输入是PointCloud2类型
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)  # 获取数据结构格式
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan  # 获取点云参数
    unpack_from = struct.Struct(fmt).unpack_from  # 创建数据解包器

    if skip_nans:  # 如果需要跳过NaN值
        if uvs:  # 如果指定了坐标
            for u, v in uvs:  # 遍历指定坐标
                p = unpack_from(data, (row_step * v) + (point_step * u))  # 解包数据
                has_nan = False  # NaN标志
                for pv in p:  # 检查每个值
                    if isnan(pv):  # 如果是NaN
                        has_nan = True  # 设置标志
                        break  # 跳出循环
                if not has_nan:  # 如果没有NaN
                    yield p  # 返回点数据
        else:  # 如果没有指定坐标
            for v in range(height):  # 遍历高度
                offset = row_step * v  # 计算行偏移
                for u in range(width):  # 遍历宽度
                    p = unpack_from(data, offset)  # 解包数据
                    has_nan = False  # NaN标志
                    for pv in p:  # 检查每个值
                        if isnan(pv):  # 如果是NaN
                            has_nan = True  # 设置标志
                            break  # 跳出循环
                    if not has_nan:  # 如果没有NaN
                        # 根据x,y,z范围过滤
                        if x_range is not None and (p[0] < x_range[0] or p[0] > x_range[1]):  # 检查x范围
                            continue  # 跳过此点
                        if y_range is not None and (p[1] < y_range[0] or p[1] > y_range[1]):  # 检查y范围
                            continue  # 跳过此点
                        if z_range is not None and (p[2] < z_range[0] or p[2] > z_range[1]):  # 检查z范围
                            continue  # 跳过此点
                        yield p  # 返回点数据
                    offset += point_step  # 更新偏移
    else:  # 如果不跳过NaN值
        if uvs:  # 如果指定了坐标
            for u, v in uvs:  # 遍历指定坐标
                yield unpack_from(data, (row_step * v) + (point_step * u))  # 返回解包的数据
        else:  # 如果没有指定坐标
            for v in range(height):  # 遍历高度
                offset = row_step * v  # 计算行偏移
                for u in range(width):  # 遍历宽度
                    yield unpack_from(data, offset)  # 返回解包的数据
                    offset += point_step  # 更新偏移


def _get_struct_fmt(is_bigendian, fields, field_names=None):  # 获取数据结构格式的函数
    fmt = '>' if is_bigendian else '<'  # 根据大小端设置格式字符串

    offset = 0  # 初始化偏移量
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):  # 遍历字段
        if offset < field.offset:  # 如果有间隔
            fmt += 'x' * (field.offset - offset)  # 添加填充
            offset = field.offset  # 更新偏移量
        if field.datatype not in _DATATYPES:  # 如果数据类型未知
            print(
                'Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)  # 打印警告
        else:  # 如果数据类型已知
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]  # 获取数据类型格式和长度
            fmt += field.count * datatype_fmt  # 添加格式字符串
            offset += field.count * datatype_length  # 更新偏移量

    return fmt  # 返回格式字符串
