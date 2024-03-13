
#include <open3d/Open3D.hpp>
using namespace open3d;// open3d名前空間を使用
t::io::RSBagReader bag_reader; // RSBagReaderオブジェクトの作成
bag_reader.Open(bag_filename); // バッグファイルを開く
auto im_rgbd = bag_reader.NextFrame(); // バッグファイルから次のフレームを読み込む
// バッグファイルの終端に達するまでループ
while (!bag_reader.IsEOF()) {
    // process im_rgbd.depth_ and im_rgbd.color_
    // im_rgbd.depth_とim_rgbd.color_を処理する（ここでは具体的な処理は省略されている）
    
    // 次のフレームを読み込む
    im_rgbd = bag_reader.NextFrame();
}
bag_reader.Close();
