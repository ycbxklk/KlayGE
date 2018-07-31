/**
 * @file MeshConverterTest.cpp
 * @author Minmin Gong
 *
 * @section DESCRIPTION
 *
 * This source file is part of KlayGE
 * For the latest info, see http://www.klayge.org
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * You may alternatively use this source under the terms of
 * the KlayGE Proprietary License (KPL). You can obtained such a license
 * from http://www.klayge.org/licensing/.
 */

#include <KlayGE/KlayGE.hpp>
#include <KlayGE/Context.hpp>
#include <KlayGE/RenderFactory.hpp>
#include <KlayGE/ResLoader.hpp>
#include <KlayGE/Texture.hpp>
#include <KFL/CXX17/filesystem.hpp>
#include <KlayGE/Mesh.hpp>
#include <KlayGE/MeshConverter.hpp>
#include <KlayGE/MeshMetadata.hpp>
#include <MeshMLLib/MeshMLLib.hpp>

#include "KlayGETests.hpp"

using namespace std;
using namespace KlayGE;

class MeshConverterTest : public testing::Test
{
public:
	void SetUp() override
	{
		ResLoader::Instance().AddPath("../../Tests/media/MeshConverter");
	}

	void RunTest(std::string_view input_name, std::string_view metadata_name, std::string const & sanity_name)
	{
		MeshMetadata metadata(metadata_name);

		MeshMLObj target(1.0f);
		int vertex_export_settings;

		MeshConverter mc;
		EXPECT_TRUE(mc.Convert(input_name, metadata, target, vertex_export_settings));

		std::vector<RenderMaterialPtr> mtls;
		std::vector<VertexElement> merged_ves;
		char all_is_index_16_bit;
		std::vector<std::vector<uint8_t>> merged_buff;
		std::vector<uint8_t> merged_indices;
		std::vector<std::string> mesh_names;
		std::vector<int32_t> mtl_ids;
		std::vector<uint32_t> mesh_lods;
		std::vector<AABBox> pos_bbs;
		std::vector<AABBox> tc_bbs;
		std::vector<uint32_t> mesh_num_vertices;
		std::vector<uint32_t> mesh_base_vertices;
		std::vector<uint32_t> mesh_num_indices;
		std::vector<uint32_t> mesh_base_indices;
		std::vector<Joint> joints;
		std::shared_ptr<AnimationActionsType> actions;
		std::shared_ptr<KeyFramesType> kfs;
		uint32_t num_frames;
		uint32_t frame_rate;
		std::vector<std::shared_ptr<AABBKeyFrames>> frame_pos_bbs;

		LoadModel(sanity_name, mtls,
			merged_ves, all_is_index_16_bit,
			merged_buff, merged_indices,
			mesh_names, mtl_ids, mesh_lods,
			pos_bbs, tc_bbs,
			mesh_num_vertices, mesh_base_vertices,
			mesh_num_indices, mesh_base_indices,
			joints, actions,
			kfs, num_frames, frame_rate,
			frame_pos_bbs);

		/*EXPECT_EQ(target->NumMipMaps(), target_sanity->NumMipMaps());
		EXPECT_EQ(target->ArraySize(), target_sanity->ArraySize());
		EXPECT_EQ(target->Type(), target_sanity->Type());
		EXPECT_EQ(target->Format(), target_sanity->Format());
		for (uint32_t m = 0; m < target->NumMipMaps(); ++ m)
		{
			EXPECT_EQ(target->Width(m), target_sanity->Width(m));
			EXPECT_EQ(target->Height(m), target_sanity->Height(m));
			EXPECT_EQ(target->Depth(m), target_sanity->Depth(m));

			EXPECT_TRUE(Compare2D(*target_sanity, 0, m, 0, 0,
				*target, 0, m, 0, 0,
				target->Width(m), target->Height(m), tolerance));
		}*/
	}
};

TEST_F(MeshConverterTest, StaticNoLod)
{
	RunTest("tree2a_lod0.obj", "tree_2a.nolod.kmeta", "tree2a_lod0.meshml");
}

TEST_F(MeshConverterTest, AnimationPassThrough)
{
	RunTest("anim.fbx", "", "anim.meshml");
}
