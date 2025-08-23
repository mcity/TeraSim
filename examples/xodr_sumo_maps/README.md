# CARLA OpenDRIVE and SUMO Maps Collection

本目录包含CARLA仿真器的OpenDRIVE地图文件和对应的SUMO网络文件，用于测试和开发OpenDRIVE到SUMO的转换器。

## 📁 目录结构

```
examples/xodr_sumo_maps/
├── carla_towns/
│   ├── xodr/               # OpenDRIVE地图文件
│   │   ├── Town01.xodr     # 基础城市地图 (486KB)
│   │   ├── Town02.xodr     # 小型城镇地图 (602KB)
│   │   ├── Town03.xodr     # 复杂城市地图 (2.2MB)
│   │   ├── Town04.xodr     # 高速公路地图 (2.0MB)
│   │   └── Town06.xodr     # 长高速公路地图 (2.0MB)
│   ├── configs/            # SUMO仿真配置文件
│   │   ├── Town01.sumocfg  # Town01 SUMO配置
│   │   ├── Town04.sumocfg  # Town04 SUMO配置
│   │   ├── Town05.sumocfg  # Town05 SUMO配置
│   │   └── carlavtypes.rou.xml  # CARLA车辆类型定义
│   └── sumo_net/           # SUMO网络文件 (待下载)
└── README.md               # 本文件
```

## 🗺️ 地图详情

### Town01 - 基础城市地图
- **类型**: 城市道路
- **大小**: 410.68 x 344.26m
- **特点**: 基础测试场景，包含交叉口和直线道路
- **适用**: 算法初期测试，连接关系验证

### Town02 - 小型城镇地图  
- **类型**: 紧凑城镇
- **大小**: 205.59 x 204.48m
- **特点**: 紧凑型布局，适合快速测试
- **适用**: 算法验证，性能测试

### Town03 - 复杂城市地图
- **类型**: 复杂城市
- **特点**: 多车道，复杂交叉口，多种道路类型
- **适用**: 复杂场景测试，交叉口处理验证

### Town04 - 高速公路地图
- **类型**: 高速公路
- **特点**: 匝道系统，多车道变化，高速场景
- **适用**: 匝道连接测试，变宽车道处理

### Town06 - 长高速公路地图
- **类型**: 长距离高速
- **特点**: 长距离高速公路，多个出入口
- **适用**: 长距离路径规划，高速场景

## 🚗 CARLA-SUMO联合仿真

### 配置文件说明
- `Town01.sumocfg`: Town01的完整SUMO仿真配置
- `Town04.sumocfg`: Town04高速公路场景配置  
- `Town05.sumocfg`: Town05城市场景配置
- `carlavtypes.rou.xml`: CARLA车辆类型定义，包含车辆蓝图映射

### 使用方法
```bash
# 运行CARLA-SUMO联合仿真
cd carla_towns/configs
sumo-gui -c Town01.sumocfg  # 使用GUI
sumo -c Town01.sumocfg      # 命令行模式
```

## 🛠️ 转换器测试

### 使用Python转换器测试
```bash
# 测试基础地图
python python_opendrive_converter_v2.py examples/xodr_sumo_maps/carla_towns/xodr/Town01.xodr output_town01.net.xml

# 测试高速公路地图（包含匝道）
python python_opendrive_converter_v2.py examples/xodr_sumo_maps/carla_towns/xodr/Town04.xodr output_town04.net.xml

# 测试复杂城市地图
python python_opendrive_converter_v2.py examples/xodr_sumo_maps/carla_towns/xodr/Town03.xodr output_town03.net.xml
```

### 与官方netconvert对比
```bash
# 使用官方netconvert转换
export SUMO_HOME=/home/mtl/.terasim/deps/sumo
netconvert --opendrive-files examples/xodr_sumo_maps/carla_towns/xodr/Town01.xodr -o official_town01.net.xml

# 对比结果
sumo-gui -n output_town01.net.xml     # Python转换结果
sumo-gui -n official_town01.net.xml   # 官方转换结果
```

## 📊 测试建议

### 入门测试顺序
1. **Town01** - 验证基础功能
2. **Town02** - 测试紧凑场景
3. **Town04** - 验证匝道和高速功能
4. **Town03** - 测试复杂交叉口
5. **Town06** - 测试长距离场景

### 重点测试方面
- ✅ **几何处理**: 直线、弧线、螺旋线
- ✅ **车道类型**: driving, entry, exit, onRamp, offRamp
- ✅ **连接关系**: 道路连接，匝道合流分流
- ✅ **交叉口**: 复杂交叉口处理
- ✅ **坐标系统**: UTM投影，地理定位

## 📚 参考资源

- [CARLA OpenDRIVE文档](https://carla.readthedocs.io/en/latest/adv_opendrive/)
- [CARLA-SUMO联合仿真](https://carla.readthedocs.io/en/latest/adv_sumo/)
- [OpenDRIVE标准](https://www.asam.net/standards/detail/opendrive/)
- [SUMO网络格式](https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html)

## 🐛 已知问题

1. **Town03复杂度**: 包含大量复杂几何，可能需要优化处理
2. **Town04匝道**: 匝道连接逻辑需要仔细验证
3. **坐标系统**: 确保地理参考正确设置

## 📈 性能指标

转换后可以使用以下指标评估：
- 边数量 vs 原始道路数量
- 节点数量 vs 交叉口数量  
- 连接数量 vs 预期连接
- 车道总数保持一致性
- 几何精度（弧线平滑度）