# plugin/CMakeLists.txt

# 1. プラグインを共有ライブラリ (.so) としてビルド
add_library(ColorControl SHARED
  ColorControl.cc
)

# 2. 自分自身のディレクトリをインクルードパスに追加 (.hh を見つけるため)
target_include_directories(ColorControl PRIVATE
  .
)

# 3. Gazebo のコアライブラリをリンク
# ※親の CMakeLists.txt で find_package されているため、そのままターゲット名が使えます
target_link_libraries(ColorControl
  ignition-gazebo6::core
)

# 4. ビルドしたプラグイン (.so) を install 先の lib ディレクトリへ配置
install(TARGETS ColorControl
  DESTINATION lib
)
