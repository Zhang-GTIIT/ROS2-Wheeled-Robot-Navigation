std_msgs/Header header
float32 convergence_error          # ICP收敛误差
float32 match_overlap_ratio        # 匹配重叠比例
int32 matched_points_count         # 匹配点数量
float32 fitness_score              # 适配度评分
float32 translation_error          # 平移误差
float32 rotation_error             # 旋转误差
float32 confidence_score           # 综合置信度评分(0-1)
bool is_converged                  # 是否收敛
int32 iteration_count              # 迭代次数
