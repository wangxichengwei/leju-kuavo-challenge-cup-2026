from modelscope import snapshot_download

# 数据集仓库ID
repo_id = "lejurobot/let_research_dataset"
# 需要下载的rosbag文件路径
target_file = "rosbag/sim/Unlabelled/scene2_Parts_Sorting/go.bag"
# 本地保存文件夹
local_save_dir = "./bag_data"

# 执行单文件下载
snapshot_download(
    repo_id=repo_id,
    local_dir=local_save_dir,
    revision="master",
    allow_file_pattern=[target_file]
)
print(f"✅ 下载完成！文件路径：{local_save_dir}/{target_file}")
