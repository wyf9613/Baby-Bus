# Python 单文件打包与一级依赖 profiling

tools/bundle_python.py 从一个 .py 入口静态收集项目 Python 依赖，生成指定的可执行
.py 文件。构建阶段不执行入口、包初始化代码或其导入模块。脚本只依赖标准库。

生成文件包含可阅读的源码字符串、资源内容和 BUNDLE_MANIFEST。运行时在临时目录
恢复原模块层级，保持包命名空间、相对导入、循环导入、__file__、资源读取和 CLI
参数。临时文件保留到进程退出，供 worker threads 和入口的 atexit callbacks 使用。
这是可执行 Python 源码打包，不生成原生机器码；运行仍需要兼容的 Python。

## 基本用法

在 Baby-Bus 根目录执行，建议显式指定 --root：

~~~powershell
python tools/bundle_python.py path/to/entry.py --root . --output build/entry_single.py
python build/entry_single.py --original-program-argument value
~~~

开启执行和模块加载统计：

~~~powershell
python tools/bundle_python.py path/to/entry.py --root . --output build/entry_profiled.py --profile-first-level
python build/entry_profiled.py
~~~

--profile 是 --profile-first-level 的别名。可选值为：

| 构建选项 | 生成文件运行行为 |
|---|---|
| 不传选项 / off | 正常执行，无 profiling 报告 |
| --profile-first-level / all | 同时统计函数/方法和模块加载 |
| --profile-first-level calls | 统计函数/方法 |
| --profile-first-level imports | 统计模块加载 |

一级依赖指入口源文件直接出现的 import/from import，以及可静态识别的字面量
importlib.import_module() / __import__()。函数内的 import 也会识别；未实际执行的
依赖仍出现在表中，耗时为零。from package import function 归属 package；
若导入对象本身是子模块，则归属该子模块。父/子依赖重叠时优先最具体的模块名。

报告在正常退出、sys.exit() 和未捕获异常时打印到 **stderr**，保留程序 stdout。
例如：

~~~text
[bundle-profile] First-level dependencies; wall time in milliseconds
dependency                                      calls     total_ms      self_ms    import_ms
planner                                           120       35.200       30.100        4.200
estimation                                         40       12.300       12.300        0.000
~~~

- calls：已观测到的 Python 函数/方法和 CPython native call 次数，包含初始化调用。
- total_ms：进入该依赖的最外层调用累计墙钟耗时，包含子调用；同依赖递归/嵌套不
  重复计入总耗时。不同一级依赖之间存在嵌套时，各自 total 可以重叠。
- self_ms：时间区间归属当前最内层一级依赖；其间未单独列为一级依赖的库调用也
  计入该上下文。它不是仅执行该文件某条 Python 指令的 CPU 时间。
- import_ms：loader 的 create_module/exec_module 阶段累计耗时，包含源码编译和
  初始化；同依赖嵌套加载避免重复计数。缓存命中的重复 import 不产生新加载时间。

三个时间列不相加。标准库有些模块由启动器先加载，入口再次导入时 import_ms=0。
新建的 Python threads 也启用统计；跨线程值是各线程墙钟耗时之和。协程/generator
计入其实际执行区间，yield/await 的挂起期不在该调用中。native 扩展没有暴露 Python
profile 事件的独立操作归属其调用上下文；异步 GPU 完成时间没有同步测量。
profiling 有额外开销，适合定位耗时；可另生成关闭 profiling 的运行版本。
os._exit()、强制终止及替换 sys.setprofile hook 不保证报告。

## 依赖范围

自动合并 --root 下的可解析 Python 源码及其递归依赖；入口所在目录支持普通
脚本的 sibling imports。支持普通包和 namespace 包，不通过执行包 __init__ 推断依赖。
可用重复 --search-path PATH 提供额外模块搜索目录。

第三方库和标准库默认保留为运行依赖，列在构建摘要和 manifest 中。可显式嵌入一个
纯 Python 库或动态模块：

~~~powershell
python tools/bundle_python.py app/main.py --root . --output build/main.py --include-module helpers.plugins
python tools/bundle_python.py app/main.py --root . --output build/main.py --include-module dreamgym
~~~

显式选择包时收集整包 Python 源码及包内数据；该包引用的未选择第三方库继续外部
安装。含 .pyd/.so 的包不能通过源码方式完整合并，构建会明确报错；用
--external numpy 等保留整包外部安装。NumPy、rclpy 及原生/系统依赖需由目标环境提供。
.pyc-only、计算式动态 imports、插件 entry points、运行时修改 __path__ 等不能普遍
静态推断。计算式导入会列入 unresolved_dynamic_imports；用 --include-module 补足
实际候选模块。--strict 拒绝未解析的动态导入及未找到的导入，适合检查闭包。

## 路径与资源

--root 应覆盖源码和依赖资源的共同项目根目录。原相对文件层级被保留，因此
Path(__file__).resolve().parents[n]、模块相对资源和 importlib.resources.files()
可继续工作。指向构建项目根内部的完整绝对字符串路径，会在源码编译时转换到临时根。
原始源码字节的 encoding/BOM/行尾被保持；路径转换不改变源代码行号。

自动资源发现识别有限的 Path(__file__)、parent/parents、resolve、路径 / 拼接、
os.path.dirname/join，以及源码中的现有文件路径字面量。不会遍历复制整个项目，
也不会自动打包隐藏目录/文件。构建摘要及 manifest 列出实际包含的资源及 SHA-256。
用 --no-auto-resources 关闭发现；计算式文件名或未识别资源需显式加入：

~~~powershell
python tools/bundle_python.py app/main.py --root . --output build/main.py --resource config/model.json --resource models/weights
python tools/bundle_python.py app/main.py --root . --output build/main.py --resource "E:/models/model.bin=models/model.bin"
~~~

项目外绝对路径、环境变量配置以及 f-string 拼接的绝对前缀保留原语义；需要通过
入口配置选择已嵌入的资源位置，或扩大 --root。不会猜测或替换任意文本中的路径。
目标资源路径不允许越出临时根。资源 symlink 保留引用的文件名并复制内容；
显式目录中的越界 symlink 会报错。改变模块/file 布局的项目源码 symlink 会报错，
请改用真实源码包，防止产生无法定位模块/资源的生成文件。

默认 **保留调用者工作目录**，使相对输出文件仍保存到调用者位置。裸
open("data.json") 依赖 cwd，可显式使用 --cwd project 或 --cwd entry，分别进入
恢复后的项目根/入口目录。此时相对写入也位于临时目录，持久输出应传入绝对路径。
生成文件是执行入口；把它 import 为库只会加载 manifest/启动支持，不会导出入口 API。

## 当前模拟环境的实用示例

~~~powershell
python tools/bundle_python.py offline/planner_sim/__main__.py --root . --output .verification/bundled_planner_sim.py --profile-first-level
python .verification/bundled_planner_sim.py --steps 20
~~~

自动收集 6 个源码文件和 2 个资源（课程 notebook、实际 policy 估计源码），
原有模拟环境及 policy 均不需要修改。NumPy/Gymnasium/Dream Gym 仍需安装；
建议使用已有 .verification/planner-sim-venv/Scripts/python.exe 执行构建与运行。
该实例已从项目外 cwd 成功运行并输出依赖耗时表。

## 验证

~~~powershell
python -B tests/test_bundle_python.py -v
~~~

用独立子进程、迁移/隐藏原源码目录验证普通与相对导入、循环引用、资源读取、
绝对地址转换、纯 Python 包、binary resources、源编码、CLI 参数、线程统计、
异常/sys.exit、外部原生库加载、资源 symlink、确定性输出，以及 profiling 的短生命周期
方法内存占用。未改动 ROS 安装、现行 policy 和源文件打包/发布工具。

实现依据：[Python importlib loader 接口](https://docs.python.org/3/library/importlib.html)、
[sys.setprofile 事件](https://docs.python.org/3/library/sys.html#sys.setprofile)。
