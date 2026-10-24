#!/usr/bin/env python3
"""Label an existing static-avoidance replay without rerunning any algorithm."""
import argparse
import json
from pathlib import Path
import re


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', type=Path)
    args = parser.parse_args()
    result = args.result.resolve()
    manifest = json.loads((result/'viewer-manifest.json').read_text())
    if not manifest['cases'] or any(c.get('collection') != 'lidar_avoidance' for c in manifest['cases']):
        raise ValueError('This presentation exporter only accepts the static avoidance collection')
    suite = json.loads((result/'suite.json').read_text())
    accepted = sum(s['avoidance_acceptance']['passed'] for s in suite['scenarios'].values())
    total = len(suite['scenarios'])
    html = (result/'viewer.html').read_text()
    html = re.sub(r'<title>.*?</title>', '<title>MVP2.1 静态雷达绕行 · 离线回放</title>', html)
    html = re.sub(r'<h1>.*?</h1>', '<h1>MVP2.1 静态雷达绕行</h1>', html)
    html = html.replace('20 Hz / 1 Hz', '20 Hz / 5 Hz')
    html = html.replace('1 Hz 候选生成与选择', '5 Hz 候选生成与选择')
    html = html.replace('当前案例 · 最大实测速度', '当前案例 · 最大仿真速度')
    html = html.replace('native 原生评分', 'MVP2.1 候选 + 雷达筛选')
    html = html.replace('native 直出', '离线绕行 / 停车对照')
    html = html.replace("{fixed:'固定掉头道路',", "{lidar_avoidance:'静态雷达绕行',fixed:'固定掉头道路',")
    html = html.replace("$('dataset-caption').textContent=", "$('dataset-caption').textContent=collection(c)==='lidar_avoidance'?'静态雷达绕行离线验证':")
    html = html.replace('当前道路集到达终点（允许回退）', '任务完成数（停车场景单独验收）')
    html = html.replace('const a=c.summary.scenario_acceptance;',
                        'const a=c.summary.avoidance_acceptance||c.summary.scenario_acceptance;')
    acceptance = """ if(c.summary.avoidance_acceptance){const a=c.summary.avoidance_acceptance;
 $('static-evidence-links').innerHTML='<a href="REPORT.md" target="_blank">验证报告</a> · <a href="avoidance-results.png" target="_blank">轨迹与速度</a> · <a href="suite.json" target="_blank">完整结果</a>';
 $('overall-label').textContent='当前案例 · 独立场景验收';$('overall').textContent=a.passed?'通过':'未通过';
 const expectation=a.expected==='complete_and_return'?'完成绕行并回归':a.expected==='clear_road_completion'?'无障碍正常行驶':a.expected==='original_stop_control'?'原停车版对照':'保护停车';
 $('outcome').innerHTML+=`<br>预期：${expectation}；障碍接触：${a.obstacle_contact?'有':'无'}；最终横向偏移 ${fmt(a.final_lateral_offset_m,3)} m。`;
 }
"""
    html, replacements = re.subn(r'^ if\(c\.summary\.obstacle_stop\)\{[^\n]*\n',
                                lambda _: acceptance, html, flags=re.MULTILINE)
    if replacements != 1:
        raise ValueError('Expected one known stop-only presentation block')
    html = re.sub(r'<p class="sub">感知的串支路.*?</p>',
        '<p class="sub">本页读取保存的实际轨迹，不运行算法。学习 policy 未启用，真实车辆未接入。'
        '规划计算尚未达到设定频率；详见 <a href="REPORT.md">验证报告</a>。</p>', html)
    notice = (f'<div class="notice">{accepted}/{total} 组满足各自验收。完成绕行和正确停车分别评价；'
              '上方任务完成计数不会把保护停车算作到达终点。本版仅覆盖固定静态障碍的宽直道离线实验。</div>')
    html = html.replace('<main>', '<main>'+notice, 1)
    target = result/'avoidance-viewer.html'
    target.write_text(html)
    print(target)


if __name__ == '__main__':
    main()
