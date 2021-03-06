/**
 * @file TexConverterTest.cpp
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
#include <KlayGE/TexConverter.hpp>
#include <KlayGE/TexMetadata.hpp>

#include "KlayGETests.hpp"

using namespace std;
using namespace KlayGE;

class TexConverterTest : public testing::Test
{
public:
	void SetUp() override
	{
		ResLoader::Instance().AddPath("../../Tests/media/TexConverter");
	}

	void RunTest(std::string_view input_name, std::string_view metadata_name, std::string const & sanity_name, float tolerance)
	{
		TexMetadata metadata(metadata_name);

		Texture::TextureType output_type;
		uint32_t output_width;
		uint32_t output_height;
		uint32_t output_depth;
		uint32_t output_num_mipmaps;
		uint32_t output_array_size;
		ElementFormat output_format;
		std::vector<ElementInitData> output_init_data;
		std::vector<uint8_t> output_data_block;

		TexConverter tc;
		EXPECT_TRUE(tc.Convert(input_name, metadata,
			output_type, output_width, output_height, output_depth, output_num_mipmaps, output_array_size, output_format,
			output_init_data, output_data_block));

		auto target = Context::Instance().RenderFactoryInstance().MakeTexture2D(output_width, output_height,
			output_num_mipmaps, output_array_size, output_format, 1, 0, EAH_CPU_Read, output_init_data);

		auto target_sanity = SyncLoadTexture(sanity_name, EAH_CPU_Read);

		EXPECT_EQ(target->NumMipMaps(), target_sanity->NumMipMaps());
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
		}
	}
};

TEST_F(TexConverterTest, NoMetadata)
{
	RunTest("lion.jpg", "", "lion.passthrough.dds", 1.0f / 255);
}

TEST_F(TexConverterTest, PassThrough)
{
	RunTest("lion.jpg", "lion.jpg.passthrough.kmeta", "lion.passthrough.dds", 1.0f / 255);
}

TEST_F(TexConverterTest, Compression)
{
	RunTest("lion.jpg", "lion.jpg.bc1.kmeta", "lion.bc1.dds", 1.0f / 255);
}

TEST_F(TexConverterTest, CompressionSRGB)
{
	RunTest("lion.jpg", "lion.jpg.bc1_srgb.kmeta", "lion.bc1_srgb.dds", 1.0f / 255);
}

TEST_F(TexConverterTest, Mip)
{
	RunTest("lion.jpg", "lion.jpg.mip.kmeta", "lion.mip.dds", 1.0f / 255);
}

TEST_F(TexConverterTest, Channel)
{
	RunTest("lion.jpg", "lion.jpg.channel.kmeta", "lion.channel.dds", 1.0f / 255);
}

TEST_F(TexConverterTest, Array)
{
	RunTest("lion.jpg", "array.kmeta", "array.dds", 1.0f / 255);
}

TEST_F(TexConverterTest, ArrayMip)
{
	RunTest("lion.jpg", "array_mip.kmeta", "array_mip.dds", 1.0f / 255);
}

TEST_F(TexConverterTest, DDSSRGB)
{
	RunTest("lion.bc1.dds", "lion.jpg.bc1_srgb.kmeta", "lion.bc1_srgb.dds", 1.0f / 255);
}

TEST_F(TexConverterTest, DDSChannel)
{
	RunTest("lion.bc1.dds", "lion.jpg.channel.kmeta", "lion.bc1_channel.dds", 1.0f / 255);
}
