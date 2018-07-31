/**
 * @file MeshConverter.cpp
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
#include <KFL/Math.hpp>
#include <KlayGE/RenderMaterial.hpp>

#include <cstring>
#include <iostream>

#if defined(KLAYGE_COMPILER_CLANGC2)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmicrosoft-enum-value" // Ignore int enum
#endif
#include <assimp/cimport.h>
#if defined(KLAYGE_COMPILER_CLANGC2)
#pragma clang diagnostic pop
#endif
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>

#include <MeshMLLib/MeshMLLib.hpp>

#include <KlayGE/MeshConverter.hpp>

using namespace std;
using namespace KlayGE;

namespace
{
	float3 Color4ToFloat3(aiColor4D const & c)
	{
		float3 v;
		v.x() = c.r;
		v.y() = c.g;
		v.z() = c.b;
		return v;
	}

	float3 AiVectorToFloat3(aiVector3D const& v)
	{
		return float3(v.x, v.y, v.z);
	}

	Quaternion AiQuatToQuat(aiQuaternion const& v)
	{
		return Quaternion(v.x, v.y, v.z, v.w);
	}

	template <typename T>
	float GetInterpTime(std::vector<T> const & vec, float time, size_t& itime_lower, size_t& itime_upper)
	{
		BOOST_ASSERT(!vec.empty());

		if (vec.size() == 1)
		{
			itime_lower = 0;
			itime_upper = 0;
			return 0;
		}

		// use @itime_upper as a hint to speed up find
		size_t vec_size = vec.size();
		size_t i = 0;
		for (i = itime_upper; i < vec_size; ++ i)
		{
			if (vec[i].first >= time)
			{
				break;
			}
		}

		if (i == 0)
		{
			itime_lower = 0;
			itime_upper = 1;
		}
		else if (i >= vec.size() - 1)
		{
			itime_lower = vec_size - 2;
			itime_upper = vec_size - 1;
		}
		else
		{
			itime_lower = i - 1;
			itime_upper = i;
		}

		float diff = vec[itime_upper].first - vec[itime_lower].first;
		return MathLib::clamp((diff == 0) ? 0 : (time - vec[itime_lower].first) / diff, 0.0f, 1.0f);
	}
}

namespace KlayGE
{
	void MeshConverter::RecursiveTransformMesh(MeshMLObj& meshml_obj, uint32_t num_lods, uint32_t lod,
		float4x4 const & parent_mat, aiNode const * node, std::vector<Mesh> const & meshes, std::vector<Mesh> const & lod0_meshes)
	{
		auto const trans_mat = MathLib::transpose(float4x4(&node->mTransformation.a1)) * parent_mat;
		auto const trans_quat = MathLib::to_quaternion(trans_mat);

		for (unsigned int n = 0; n < node->mNumMeshes; ++ n)
		{
			auto const & mesh = meshes[node->mMeshes[n]];

			int mesh_id;
			if (lod == 0)
			{
				mesh_id = meshml_obj.AllocMesh();
				meshml_obj.SetMesh(mesh_id, mesh.mtl_id, mesh.name, num_lods);
			}
			else
			{
				mesh_id = -1;
				for (size_t i = 0; i < lod0_meshes.size(); ++ i)
				{
					if (lod0_meshes[i].mtl_id == mesh.mtl_id)
					{
						mesh_id = static_cast<int>(i);
						break;
					}
				}
				BOOST_ASSERT(mesh_id != -1);
			}

			for (unsigned int ti = 0; ti < mesh.indices.size(); ti += 3)
			{
				int tri_id = meshml_obj.AllocTriangle(mesh_id, lod);
				meshml_obj.SetTriangle(mesh_id, lod, tri_id, mesh.indices[ti + 0],
					mesh.indices[ti + 1], mesh.indices[ti + 2]);
			}

			for (unsigned int vi = 0; vi < mesh.positions.size(); ++ vi)
			{
				int vertex_id = meshml_obj.AllocVertex(mesh_id, lod);

				std::vector<float3> texcoords;
				for (unsigned int tci = 0; tci < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++ tci)
				{
					if (mesh.has_texcoord[tci])
					{
						texcoords.push_back(mesh.texcoords[tci][vi]);
					}
				}

				auto const pos = MathLib::transform_coord(mesh.positions[vi], trans_mat);
				if (mesh.has_tangent_frame)
				{
					auto const quat = mesh.tangent_quats[vi] * trans_quat;
					meshml_obj.SetVertex(mesh_id, lod, vertex_id, pos, quat, 2, texcoords);
				}
				else
				{
					auto const normal = MathLib::transform_normal(mesh.normals[vi], trans_mat);
					meshml_obj.SetVertex(mesh_id, lod, vertex_id, pos, normal, 2, texcoords);
				}
			}

			for (unsigned int wi = 0; wi < mesh.joint_binding.size(); ++ wi)
			{
				auto binding = mesh.joint_binding[wi];
				int bind_id = meshml_obj.AllocJointBinding(mesh_id, lod, binding.vertex_id);
				meshml_obj.SetJointBinding(mesh_id, lod, binding.vertex_id, bind_id, binding.joint_id, binding.weight);
			}
		}

		for (unsigned int i = 0; i < node->mNumChildren; ++ i)
		{
			RecursiveTransformMesh(meshml_obj, num_lods, lod, trans_mat, node->mChildren[i], meshes, lod0_meshes);
		}
	}

	void MeshConverter::ConvertMaterials(MeshMLObj& meshml_obj, aiScene const * scene)
	{
		for (unsigned int mi = 0; mi < scene->mNumMaterials; ++ mi)
		{
			int mtl_id = meshml_obj.AllocMaterial();

			std::string name;
			float3 albedo(0, 0, 0);
			float metalness = 0;
			float shininess = 1;
			float3 emissive(0, 0, 0);
			float opacity = 1;
			bool transparent = false;
			bool two_sided = false;

			aiString ai_name;
			aiColor4D ai_albedo;
			float ai_opacity;
			float ai_shininess;
			aiColor4D ai_emissive;
			int ai_two_sided;

			auto mtl = scene->mMaterials[mi];

			if (AI_SUCCESS == aiGetMaterialString(mtl, AI_MATKEY_NAME, &ai_name))
			{
				name = ai_name.C_Str();
			}

			if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &ai_albedo))
			{
				albedo = Color4ToFloat3(ai_albedo);
			}
			if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &ai_emissive))
			{
				emissive = Color4ToFloat3(ai_emissive);
			}

			unsigned int max = 1;
			if (AI_SUCCESS == aiGetMaterialFloatArray(mtl, AI_MATKEY_OPACITY, &ai_opacity, &max))
			{
				opacity = ai_opacity;
			}

			max = 1;
			if (AI_SUCCESS == aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &ai_shininess, &max))
			{
				shininess = ai_shininess;

				max = 1;
				float strength;
				if (AI_SUCCESS == aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max))
				{
					shininess *= strength;
				}
			}
			shininess = MathLib::clamp(shininess, 1.0f, MAX_SHININESS);

			if ((opacity < 1) || (aiGetMaterialTextureCount(mtl, aiTextureType_OPACITY) > 0))
			{
				transparent = true;
			}

			max = 1;
			if (AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &ai_two_sided, &max))
			{
				two_sided = ai_two_sided ? true : false;
			}

			meshml_obj.SetMaterial(mtl_id, name, float4(albedo.x(), albedo.y(), albedo.z(), opacity),
				metalness, Shininess2Glossiness(shininess),
				emissive, transparent, 0, false, two_sided);

			unsigned int count = aiGetMaterialTextureCount(mtl, aiTextureType_DIFFUSE);
			if (count > 0)
			{
				aiString str;
				aiGetMaterialTexture(mtl, aiTextureType_DIFFUSE, 0, &str, 0, 0, 0, 0, 0, 0);

				meshml_obj.SetTextureSlot(mtl_id, MeshMLObj::Material::TS_Albedo, str.C_Str());
			}

			count = aiGetMaterialTextureCount(mtl, aiTextureType_SHININESS);
			if (count > 0)
			{
				aiString str;
				aiGetMaterialTexture(mtl, aiTextureType_SHININESS, 0, &str, 0, 0, 0, 0, 0, 0);

				meshml_obj.SetTextureSlot(mtl_id, MeshMLObj::Material::TS_Glossiness, str.C_Str());
			}

			count = aiGetMaterialTextureCount(mtl, aiTextureType_EMISSIVE);
			if (count > 0)
			{
				aiString str;
				aiGetMaterialTexture(mtl, aiTextureType_EMISSIVE, 0, &str, 0, 0, 0, 0, 0, 0);

				meshml_obj.SetTextureSlot(mtl_id, MeshMLObj::Material::TS_Emissive, str.C_Str());
			}

			count = aiGetMaterialTextureCount(mtl, aiTextureType_NORMALS);
			if (count > 0)
			{
				aiString str;
				aiGetMaterialTexture(mtl, aiTextureType_NORMALS, 0, &str, 0, 0, 0, 0, 0, 0);

				meshml_obj.SetTextureSlot(mtl_id, MeshMLObj::Material::TS_Normal, str.C_Str());
			}

			count = aiGetMaterialTextureCount(mtl, aiTextureType_HEIGHT);
			if (count > 0)
			{
				aiString str;
				aiGetMaterialTexture(mtl, aiTextureType_HEIGHT, 0, &str, 0, 0, 0, 0, 0, 0);

				meshml_obj.SetTextureSlot(mtl_id, MeshMLObj::Material::TS_Height, str.C_Str());
			}
		}
	}

	void MeshConverter::BuildMeshData(std::vector<Mesh>& meshes, int& vertex_export_settings, JointsMap const& joint_nodes, aiScene const * scene,
		bool update_center, float3& center, MeshMetadata const & input_metadata)
	{
		vertex_export_settings = MeshMLObj::VES_None;
		for (unsigned int mi = 0; mi < scene->mNumMeshes; ++ mi)
		{
			aiMesh const * mesh = scene->mMeshes[mi];

			meshes[mi].mtl_id = mesh->mMaterialIndex;
			meshes[mi].name = mesh->mName.C_Str();

			auto& indices = meshes[mi].indices;
			for (unsigned int fi = 0; fi < mesh->mNumFaces; ++ fi)
			{
				BOOST_ASSERT(3 == mesh->mFaces[fi].mNumIndices);

				indices.push_back(mesh->mFaces[fi].mIndices[0]);
				indices.push_back(mesh->mFaces[fi].mIndices[1]);
				indices.push_back(mesh->mFaces[fi].mIndices[2]);
			}

			bool has_normal = (mesh->mNormals != nullptr);
			bool has_tangent = (mesh->mTangents != nullptr);
			bool has_binormal = (mesh->mBitangents != nullptr);
			auto& has_texcoord = meshes[mi].has_texcoord;
			uint32_t first_texcoord = AI_MAX_NUMBER_OF_TEXTURECOORDS;
			for (unsigned int tci = 0; tci < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++ tci)
			{
				has_texcoord[tci] = (mesh->mTextureCoords[tci] != nullptr);
				if (has_texcoord[tci] && (AI_MAX_NUMBER_OF_TEXTURECOORDS == first_texcoord))
				{
					first_texcoord = tci;
				}
			}

			auto& positions = meshes[mi].positions;
			auto& normals = meshes[mi].normals;
			std::vector<float3> tangents(mesh->mNumVertices);
			std::vector<float3> binormals(mesh->mNumVertices);
			auto& texcoords = meshes[mi].texcoords;
			positions.resize(mesh->mNumVertices);
			normals.resize(mesh->mNumVertices);
			for (unsigned int tci = 0; tci < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++ tci)
			{
				texcoords[tci].resize(mesh->mNumVertices);
			}
			for (unsigned int vi = 0; vi < mesh->mNumVertices; ++ vi)
			{
				positions[vi] = MathLib::transform_coord(float3(&mesh->mVertices[vi].x), input_metadata.Transform()) - center;

				if (has_normal)
				{
					normals[vi] = MathLib::transform_normal(float3(&mesh->mNormals[vi].x), input_metadata.TransformIT());
				}
				if (has_tangent)
				{
					tangents[vi] = MathLib::transform_normal(float3(&mesh->mTangents[vi].x), input_metadata.Transform());
				}
				if (has_binormal)
				{
					binormals[vi] = MathLib::transform_normal(float3(&mesh->mBitangents[vi].x), input_metadata.Transform());
				}

				for (unsigned int tci = 0; tci < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++ tci)
				{
					if (has_texcoord[tci])
					{
						BOOST_ASSERT(mesh->mTextureCoords[tci] != nullptr);
						KLAYGE_ASSUME(mesh->mTextureCoords[tci] != nullptr);

						texcoords[tci][vi] = float3(&mesh->mTextureCoords[tci][vi].x);
					}
				}
			}

			if (!has_normal)
			{
				MathLib::compute_normal(normals.begin(), indices.begin(), indices.end(), positions.begin(), positions.end());

				has_normal = true;
			}

			auto& tangent_quats = meshes[mi].tangent_quats;
			tangent_quats.resize(mesh->mNumVertices);
			if ((!has_tangent || !has_binormal) && (first_texcoord != AI_MAX_NUMBER_OF_TEXTURECOORDS))
			{
				MathLib::compute_tangent(tangents.begin(), binormals.begin(), indices.begin(), indices.end(),
					positions.begin(), positions.end(), texcoords[first_texcoord].begin(), normals.begin());

				for (size_t i = 0; i < positions.size(); ++ i)
				{
					tangent_quats[i] = MathLib::to_quaternion(tangents[i], binormals[i], normals[i], 8);
				}

				has_tangent = true;
			}

			meshes[mi].has_normal = has_normal;
			meshes[mi].has_tangent_frame = has_tangent || has_binormal;

			if (has_tangent || has_binormal)
			{
				vertex_export_settings |= MeshMLObj::VES_TangentQuat;
			}
			if (first_texcoord != AI_MAX_NUMBER_OF_TEXTURECOORDS)
			{
				vertex_export_settings |= MeshMLObj::VES_Texcoord;
			}

			for (unsigned int bi = 0; bi < mesh->mNumBones; ++ bi)
			{
				aiBone* bone = mesh->mBones[bi];
				auto iter = joint_nodes.find(bone->mName.C_Str());
				BOOST_ASSERT_MSG(iter != joint_nodes.end(), "Joint not found!");

				int joint_id = iter->second.id;
				for (unsigned int wi = 0; wi < bone->mNumWeights; ++ wi)
				{
					int vertex_id = bone->mWeights[wi].mVertexId;
					float weight = bone->mWeights[wi].mWeight;
					meshes[mi].joint_binding.push_back({ joint_id, vertex_id, weight });
				}
			}
		}

		if (input_metadata.AutoCenter() && update_center)
		{
			auto aabb = MathLib::compute_aabbox(meshes[0].positions.begin(), meshes[0].positions.end());
			for (unsigned int mi = 0; mi < scene->mNumMeshes; ++ mi)
			{
				auto aabb_mesh = MathLib::compute_aabbox(meshes[mi].positions.begin(), meshes[mi].positions.end());
				aabb = AABBox(MathLib::minimize(aabb.Min(), aabb_mesh.Min()), MathLib::maximize(aabb.Max(), aabb_mesh.Max()));
			}
			center = aabb.Center();
			for (unsigned int mi = 0; mi < scene->mNumMeshes; ++ mi)
			{
				for (auto& pos : meshes[mi].positions)
				{
					pos -= center;
				}
			}
		}

		for (unsigned int mi = 0; mi < scene->mNumMeshes; ++ mi)
		{
			if ((vertex_export_settings & MeshMLObj::VES_TangentQuat) && !meshes[mi].has_tangent_frame)
			{
				meshes[mi].has_tangent_frame = true;
			}
			if ((vertex_export_settings & MeshMLObj::VES_Texcoord) && !meshes[mi].has_texcoord[0])
			{
				meshes[mi].has_texcoord[0] = true;
			}
		}
	}

	void MeshConverter::BuildJoints(MeshMLObj& meshml_obj, JointsMap& joint_nodes, aiScene const * scene)
	{
		std::function<void(aiNode const *, float4x4 const &)> build_bind_matrix =
			[&build_bind_matrix, &joint_nodes, scene](aiNode const * meshNode, float4x4 const & parent_mat)
		{
			float4x4 const mesh_trans = MathLib::transpose(float4x4(&meshNode->mTransformation.a1)) * parent_mat;
			for (unsigned int i = 0; i < meshNode->mNumMeshes; ++ i)
			{
				aiMesh const * mesh = scene->mMeshes[meshNode->mMeshes[i]];
				for (unsigned int ibone = 0; ibone < mesh->mNumBones; ++ ibone)
				{
					aiBone const * bone = mesh->mBones[ibone];
					Joint joint;
					joint.name = bone->mName.C_Str();
					joint.bone_to_mesh = MathLib::inverse(MathLib::transpose(float4x4(&bone->mOffsetMatrix.a1))) * mesh_trans;
					joint_nodes[joint.name] = joint;
				}
			}

			for (unsigned int i = 0; i < meshNode->mNumChildren; ++ i)
			{
				build_bind_matrix(meshNode->mChildren[i], mesh_trans);
			}
		};

		std::function<bool(aiNode const *)> mark_joint_nodes = [&mark_joint_nodes, &joint_nodes](aiNode const * root)
		{
			std::string name = root->mName.C_Str();
			bool child_has_bone = false;

			auto iter = joint_nodes.find(name);
			if (iter != joint_nodes.end())
			{
				child_has_bone = true;
			}

			for (unsigned int i = 0; i < root->mNumChildren; ++ i)
			{
				child_has_bone = mark_joint_nodes(root->mChildren[i]) || child_has_bone;
			}

			if (child_has_bone && (iter == joint_nodes.end()))
			{
				Joint joint;
				joint.name = name;
				joint.bone_to_mesh = float4x4::Identity();
				joint_nodes[name] = joint;
			}

			return child_has_bone;
		};

		std::function<void(aiNode const *, int)> alloc_joints =
			[&meshml_obj, &joint_nodes, &alloc_joints](aiNode const * root, int parent_id)
		{
			std::string name = root->mName.C_Str();
			int joint_id = -1;
			auto iter = joint_nodes.find(name);
			if (iter != joint_nodes.end())
			{
				joint_id = meshml_obj.AllocJoint();
				iter->second.id = joint_id;
				iter->second.local_matrix = MathLib::transpose(float4x4(&root->mTransformation.a1));

				meshml_obj.SetJoint(joint_id, name, parent_id, iter->second.bone_to_mesh);
			}

			for (unsigned int i = 0; i < root->mNumChildren; ++ i)
			{
				alloc_joints(root->mChildren[i], joint_id);
			}
		};

		build_bind_matrix(scene->mRootNode, float4x4::Identity());
		mark_joint_nodes(scene->mRootNode);
		alloc_joints(scene->mRootNode, -1);
	}

	void MeshConverter::BuildActions(MeshMLObj& meshml_obj, JointsMap const & joint_nodes, aiScene const * scene)
	{
		struct Animation
		{
			std::string name;
			int frame_num;
			std::map<int/*joint_id*/, JointResampledKeyframe> resampled_frames;
		};

		std::vector<Animation> animations;

		int const resample_fps = 25;
		// for actions
		for (unsigned int ianim = 0; ianim < scene->mNumAnimations; ++ ianim)
		{
			aiAnimation const * cur_anim = scene->mAnimations[ianim];
			float duration = static_cast<float>(cur_anim->mDuration / cur_anim->mTicksPerSecond);
			Animation anim;
			anim.name = cur_anim->mName.C_Str();
			anim.frame_num = static_cast<int>(ceilf(duration * resample_fps));
			if (anim.frame_num == 0)
			{
				anim.frame_num = 1;
			}

			// import joints animation
			for (unsigned int ichannel = 0; ichannel < cur_anim->mNumChannels; ++ ichannel)
			{
				aiNodeAnim const * cur_joint = cur_anim->mChannels[ichannel];
				int joint_id = -1;
				auto iter = joint_nodes.find(cur_joint->mNodeName.C_Str());
				if (iter != joint_nodes.end())
				{
					joint_id = iter->second.id;
				}

				// NOTE: ignore animation if node is not joint
				if (joint_id > 0)
				{
					JointKeyframe kf;
					for (unsigned int i = 0; i < cur_joint->mNumPositionKeys; ++ i)
					{
						auto const & p = cur_joint->mPositionKeys[i];
						kf.pos.push_back(std::make_pair(static_cast<float>(p.mTime), AiVectorToFloat3(p.mValue)));
					}

					for (unsigned int i = 0; i < cur_joint->mNumRotationKeys; ++ i)
					{
						auto const & p = cur_joint->mRotationKeys[i];
						kf.quats.push_back(std::make_pair(static_cast<float>(p.mTime), AiQuatToQuat(p.mValue)));
					}

					for (unsigned int i = 0; i < cur_joint->mNumScalingKeys; ++ i)
					{
						auto const & p = cur_joint->mScalingKeys[i];
						kf.scale.push_back(std::make_pair(static_cast<float>(p.mTime), AiVectorToFloat3(p.mValue)));
					}

					// resample
					JointResampledKeyframe resampled_kf;
					this->ResampleJointTransform(0, anim.frame_num, static_cast<float>(cur_anim->mTicksPerSecond / resample_fps),
						kf, resampled_kf);

					anim.resampled_frames[joint_id] = resampled_kf;
				}
			}

			for (auto const & joint : joint_nodes)
			{
				int joint_id = joint.second.id;
				if (anim.resampled_frames.find(joint_id) == anim.resampled_frames.end())
				{
					ResampledTransform default_tf;
					default_tf.frame = 0;
					Quaternion quat;
					float3 pos;
					float3 scale;
					MathLib::decompose(scale, quat, pos, joint.second.local_matrix);
					default_tf.bind_real = quat;
					default_tf.bind_dual = MathLib::quat_trans_to_udq(quat, pos);
					if (MathLib::SignBit(default_tf.bind_real.w()) < 0)
					{
						default_tf.bind_real = -default_tf.bind_real;
						default_tf.bind_dual = -default_tf.bind_dual;
					}
					default_tf.scale = scale.x();
					anim.resampled_frames[joint_id].push_back(default_tf);
				}
			}

			animations.push_back(anim);
		}

		int action_frame_offset = 0;
		for (auto const & anim : animations)
		{
			int action_id = meshml_obj.AllocAction();
			meshml_obj.SetAction(action_id, anim.name, action_frame_offset, action_frame_offset + anim.frame_num);

			for (auto const & frame : anim.resampled_frames)
			{
				int joint_id = frame.first;
				int kfs_id = meshml_obj.AllocKeyframeSet();
				meshml_obj.SetKeyframeSet(kfs_id, joint_id);

				for (auto const & resampled : frame.second)
				{
					int shifted_frame = resampled.frame + action_frame_offset;
					int kf_id = meshml_obj.AllocKeyframe(kfs_id);

					meshml_obj.SetKeyframe(kfs_id, kf_id, shifted_frame, resampled.bind_real * resampled.scale, resampled.bind_dual);
				}
			}

			action_frame_offset = action_frame_offset + anim.frame_num;
		}

		meshml_obj.FrameRate(resample_fps);
		meshml_obj.NumFrames(action_frame_offset);
	}

	void MeshConverter::ResampleJointTransform(int start_frame, int end_frame, float fps_scale,
		JointKeyframe const & okf, JointResampledKeyframe& rkf)
	{
		size_t i_pos = 0;
		size_t i_rot = 0;
		size_t i_scale = 0;
		for (int i = start_frame; i < end_frame; ++ i)
		{
			float time = i * fps_scale;
			size_t prev_i = 0;
			float fraction = 0.0f;
			float3 scale_resampled(1, 1, 1);
			Quaternion bind_real_resampled(0, 0, 0, 1);
			Quaternion bind_dual_resampled(0, 0, 0, 0);

			if (!okf.scale.empty())
			{
				fraction = GetInterpTime(okf.scale, time, prev_i, i_scale);
				scale_resampled = MathLib::lerp(okf.scale[prev_i].second, okf.scale[i_scale].second, fraction);
			}
			if (!okf.quats.empty())
			{
				fraction = GetInterpTime(okf.quats, time, prev_i, i_rot);
				bind_real_resampled = MathLib::slerp(okf.quats[prev_i].second, okf.quats[i_rot].second, fraction);
			}
			if (!okf.pos.empty())
			{
				fraction = GetInterpTime(okf.pos, time, prev_i, i_pos);

				auto bind_dual_prev_i = MathLib::quat_trans_to_udq(okf.quats[prev_i].second, okf.pos[prev_i].second);
				auto bind_dual_i_pos = MathLib::quat_trans_to_udq(okf.quats[i_rot].second, okf.pos[i_pos].second);

				auto bind_dq_resampled = MathLib::sclerp(okf.quats[prev_i].second, bind_dual_prev_i,
					okf.quats[i_rot].second, bind_dual_i_pos,
					fraction);

				bind_dual_resampled = bind_dq_resampled.second;
			}

			if (MathLib::SignBit(bind_real_resampled.w()) < 0)
			{
				bind_real_resampled = -bind_real_resampled;
				bind_dual_resampled = -bind_dual_resampled;
			}

			rkf.push_back({ i, bind_real_resampled, bind_dual_resampled, scale_resampled.x() });
		}
	}

	bool MeshConverter::Convert(std::string_view in_name, MeshMetadata const & input_metadata,
		MeshMLObj& meshml_obj, int& vertex_export_settings)
	{
		auto ai_property_store_deleter = [](aiPropertyStore* props)
		{
			aiReleasePropertyStore(props);
		};

		std::unique_ptr<aiPropertyStore, decltype(ai_property_store_deleter)> props(aiCreatePropertyStore(), ai_property_store_deleter);
		aiSetImportPropertyInteger(props.get(), AI_CONFIG_IMPORT_TER_MAKE_UVS, 1);
		aiSetImportPropertyFloat(props.get(), AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 80);
		aiSetImportPropertyInteger(props.get(), AI_CONFIG_PP_SBP_REMOVE, 0);

		aiSetImportPropertyInteger(props.get(), AI_CONFIG_GLOB_MEASURE_TIME, 1);

		unsigned int ppsteps = aiProcess_JoinIdenticalVertices // join identical vertices/ optimize indexing
			| aiProcess_ValidateDataStructure // perform a full validation of the loader's output
			| aiProcess_RemoveRedundantMaterials // remove redundant materials
			| aiProcess_FindInstances; // search for instanced meshes and remove them by references to one master

		uint32_t const num_lods = static_cast<uint32_t>(input_metadata.NumLods());

		auto ai_scene_deleter = [](aiScene const * scene)
		{
			aiReleaseImport(scene);
		};

		std::vector<std::shared_ptr<aiScene const>> scenes(num_lods);
		for (uint32_t lod = 0; lod < num_lods; ++ lod)
		{
			std::string const file_name = std::string((lod == 0) ? in_name : input_metadata.LodFileName(lod));
			scenes[lod].reset(aiImportFileExWithProperties(file_name.c_str(),
				ppsteps // configurable pp steps
				| aiProcess_GenSmoothNormals // generate smooth normal vectors if not existing
				| aiProcess_Triangulate // triangulate polygons with more than 3 edges
				| aiProcess_ConvertToLeftHanded // convert everything to D3D left handed space
				| aiProcess_FixInfacingNormals, // find normals facing inwards and inverts them
				nullptr, props.get()), ai_scene_deleter);

			if (!scenes[lod])
			{
				cout << "Assimp: Import file error: " << aiGetErrorString() << '\n';
				return false;
			}
		}

		ConvertMaterials(meshml_obj, scenes[0].get());

		std::vector<std::vector<Mesh>> meshes(num_lods);
		for (uint32_t lod = 0; lod < num_lods; ++ lod)
		{
			meshes[lod].resize(scenes[lod]->mNumMeshes);
		}

		JointsMap joint_nodes;

		BuildJoints(meshml_obj, joint_nodes, scenes[0].get());

		vertex_export_settings = 0;
		float3 center(0, 0, 0);
		for (uint32_t lod = 0; lod < num_lods; ++ lod)
		{
			BuildMeshData(meshes[lod], vertex_export_settings, joint_nodes, scenes[lod].get(), lod == 0, center, input_metadata);
			RecursiveTransformMesh(meshml_obj, num_lods, lod, float4x4::Identity(), scenes[lod]->mRootNode, meshes[lod], meshes[0]);
		}
		BuildActions(meshml_obj, joint_nodes, scenes[0].get());

		return true;
	}
}
