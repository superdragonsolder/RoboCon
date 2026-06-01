#!/usr/bin/env python3
"""
将 py_trees 行为树导出为 Groot2 (BehaviorTree.CPP v4) 兼容的 XML 文件。

用法：
    python3 generate_groot_xml.py                    # 使用默认树输出到当前目录
    python3 generate_groot_xml.py -o r2_tree.xml     # 指定输出文件
    python3 generate_groot_xml.py -o r2_tree.xml -p  # 输出后自动打印

在 Groot2 中：
    1. 打开 Groot2 → Load Tree → 选择生成的 .xml 文件
    2. 即可查看树的结构拓扑（静态）
"""

import argparse
import os
import sys
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

import py_trees
import py_trees.composites
import py_trees.decorators

# ---- 模拟 py_trees 行为树结构（不依赖 ROS/硬件） ----
# 这里用占位节点构建与 r2_behavior_tree.py 完全一致的树拓扑


class _Placeholder(py_trees.behaviour.Behaviour):
    """占位叶节点，仅用于导出 XML 结构。"""
    def update(self):
        return py_trees.common.Status.SUCCESS


def build_tree_for_export() -> py_trees.behaviour.Behaviour:
    """构建与 R2BehaviorTreeHost._build_tree() 一致的树结构（含看门狗）。"""
    定位保护 = py_trees.composites.Selector(
        name="定位保护",
        memory=False,
        children=[
            _Placeholder("检查雷达重定位"),
            _Placeholder("底盘紧急制动"),
        ],
    )

    导航执行流 = py_trees.composites.Sequence(
        name="导航执行流",
        memory=True,
        children=[
            _Placeholder("设置相对目标"),
            _Placeholder("计算PID速度"),
            _Placeholder("是否到达目标"),
        ],
    )

    # ── 预定义动作子序列（可独立调用）──
    移动动作库 = py_trees.composites.Sequence(
        name="常用移动动作",
        memory=True,
        children=[
            _Placeholder("前进"),
            _Placeholder("后退"),
            _Placeholder("左移"),
            _Placeholder("右移"),
            _Placeholder("左转90度"),
            _Placeholder("右转90度"),
        ],
    )

    root = py_trees.composites.Sequence(
        name="R2导航根序列",
        memory=True,
        children=[
            _Placeholder("更新当前位姿"),
            定位保护,
            _Placeholder("看门狗检查"),
            导航执行流,
            移动动作库,
        ],
    )

    return root


def extract_leaf_names(root: py_trees.behaviour.Behaviour) -> List[str]:
    """递归收集所有叶节点名称（去重）。"""
    names: List[str] = []

    def _walk(node):
        if not hasattr(node, 'children') or not node.children:
            names.append(node.name)
        else:
            for child in node.children:
                _walk(child)

    _walk(root)
    # 保持顺序去重
    seen = set()
    result = []
    for n in names:
        if n not in seen:
            result.append(n)
            seen.add(n)
    return result


def build_groot_xml(root: py_trees.behaviour.Behaviour) -> ET.Element:
    """将 py_trees 行为树转换为 BehaviorTree.CPP v4 XML ElementTree。

    Groot2 BTCPP_format="4" 格式：
      - Sequence → <Sequence name="...">
      - Selector → <Fallback name="...">
      - Action   → <Action ID="..."/>
      - Condition → <Condition ID="..."/>
    """

    # 收集所有叶子节点名作为 TreeNodesModel
    leaf_names = extract_leaf_names(root)

    # ---- 构建 <root> ----
    root_el = ET.Element("root", {"BTCPP_format": "4"})

    # ---- 构建 <BehaviorTree> ----
    bt_el = ET.SubElement(root_el, "BehaviorTree", {"ID": "R2Navigation"})

    def _convert(node: py_trees.behaviour.Behaviour) -> ET.Element:
        """递归将 py_trees 节点转为 Groot XML 元素。"""
        if isinstance(node, py_trees.composites.Sequence):
            el = ET.Element("Sequence", {"name": node.name})
            for child in node.children:
                el.append(_convert(child))
            return el
        elif isinstance(node, py_trees.composites.Selector):
            el = ET.Element("Fallback", {"name": node.name})
            for child in node.children:
                el.append(_convert(child))
            return el
        elif isinstance(node, py_trees.composites.Parallel):
            el = ET.Element("Parallel", {"name": node.name})
            for child in node.children:
                el.append(_convert(child))
            return el
        elif isinstance(node, py_trees.decorators.Decorator):
            el = ET.Element("Decorator", {
                "ID": node.name.replace(" ", "_"),
                "name": node.name,
            })
            el.append(_convert(node.children[0]))
            return el
        else:
            # 叶节点：根据命名惯例判定 Action / Condition
            name = node.name
            if "检查" in name or "是否" in name:
                return ET.Element("Condition", {"ID": name, "name": name})
            else:
                return ET.Element("Action", {"ID": name, "name": name})

    bt_el.append(_convert(root))

    # ---- 构建 <TreeNodesModel> ----
    model_el = ET.SubElement(root_el, "TreeNodesModel")

    # 注册所有叶节点
    for name in leaf_names:
        if "检查" in name or "是否" in name:
            ET.SubElement(model_el, "Condition", {
                "ID": name,
                "editable": "true",
            })
        else:
            ET.SubElement(model_el, "Action", {
                "ID": name,
                "editable": "true",
            })

    return root_el


def pretty_xml(element: ET.Element) -> str:
    """格式化 XML 输出。"""
    raw = ET.tostring(element, encoding="unicode")
    dom = minidom.parseString(raw)
    return dom.toprettyxml(indent="  ")


def main():
    parser = argparse.ArgumentParser(
        description="将 py_trees 行为树导出为 Groot2 XML"
    )
    parser.add_argument(
        "-o", "--output",
        default="r2_behavior_tree_groot.xml",
        help="输出 XML 文件路径 (默认: r2_behavior_tree_groot.xml)",
    )
    parser.add_argument(
        "-p", "--print",
        action="store_true",
        help="同时打印 XML 到控制台",
    )
    args = parser.parse_args()

    # 构建树
    root = build_tree_for_export()

    # 打印静态结构（控制台 Unicode）
    print("========== py_trees 静态结构 ==========")
    print(py_trees.display.unicode_tree(root, show_status=False))

    # 生成 Groot XML
    xml_root = build_groot_xml(root)
    xml_str = pretty_xml(xml_root)

    # 写入文件
    out_path = os.path.abspath(args.output)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(xml_str)
    print(f"\n✅ Groot2 XML 已导出: {out_path}")

    if args.print:
        print("\n========== Groot2 XML 内容 ==========")
        print(xml_str)

    print("""
📖 在 Groot2 中查看:
   1. 打开 Groot2 应用
   2. 菜单: File → Load Tree (或 Ctrl+O)
   3. 选择刚刚生成的 XML 文件
   4. 即可看到完整的树结构
""")


if __name__ == "__main__":
    main()
