/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MESH_NODE_H
#define MESH_NODE_H

#include <rviz/mesh_loader.h>
#ifndef Q_MOC_RUN
#include <resource_retriever/retriever.h>
#endif
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h> // hack to get camera position 

#include <OgreSceneManager.h>
#include <OgreSubEntity.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreRoot.h>
#include <OgreFrameListener.h>


namespace spencer_tracking_rviz_plugin {
    class MeshNode : public Ogre::FrameListener {
    public:
        MeshNode(rviz::DisplayContext* displayContext, Ogre::SceneNode* parentNode, const std::string& meshResource, Ogre::Vector3 position = Ogre::Vector3::ZERO)
        : m_sceneManager(displayContext->getSceneManager()), m_displayContext(displayContext), m_meshResource(meshResource)
        {
            m_cameraFacing = false;
            m_sceneNode = parentNode->createChildSceneNode();
            m_sceneNode->setVisible(false);

            // Load mesh
            assert(!rviz::loadMeshFromResource(meshResource).isNull());

            // Create scene entity
            std::stringstream ss;
            static int counter = 0;
            ss << "gender_symbol_" << counter++;
            std::string id = ss.str();

            m_entity = m_sceneManager->createEntity(id, meshResource);
            m_sceneNode->attachObject(m_entity);

            // set up material
            ss << "Material";
            // Ogre::MaterialPtr default_material = Ogre::MaterialManager::getSingleton().create( ss.str(), "rviz" );
            Ogre::MaterialPtr default_material = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            default_material->setReceiveShadows(false);
            default_material->getTechnique(0)->setLightingEnabled(true);
            default_material->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );
            m_materials.insert( default_material );
            m_entity->setMaterial( default_material );

            setPosition(position);
            setVisible(true);

            // For camera-facing meshes
            Ogre::Root::getSingleton().addFrameListener(this);
        }

        virtual ~MeshNode() {
            Ogre::Root::getSingleton().removeFrameListener(this);
            m_sceneManager->destroyEntity(m_entity);

            // destroy all the materials we've created
            std::set<Ogre::MaterialPtr>::iterator it;
            for(it = m_materials.begin(); it != m_materials.end(); it++ )
            {
                Ogre::MaterialPtr material = *it;
                if (!material.isNull())
                {
                  material->unload();
                  Ogre::MaterialManager::getSingleton().remove(material->getName());
                }
            }
            m_materials.clear();
            m_sceneManager->destroySceneNode(m_sceneNode->getName());
        }

        void setOrientation(const Ogre::Quaternion& orientation) {
            m_orientation = orientation;
        }

        void setPosition(const Ogre::Vector3& position) {
            m_sceneNode->setPosition(position);
        }

        void setScale(const float scaleFactor) {
            m_sceneNode->setScale(Ogre::Vector3(scaleFactor, scaleFactor, scaleFactor));
        }

        void setVisible(bool visible) {
            m_sceneNode->setVisible(visible, true);
        }

        void setCameraFacing(bool cameraFacing) {
            m_cameraFacing = cameraFacing;
        }

        void setColor(const Ogre::ColourValue& c) {
            Ogre::SceneBlendType blending;
            bool depth_write;

            if ( c.a < 0.9998 )
            {
              blending = Ogre::SBT_TRANSPARENT_ALPHA;
              depth_write = false;
            }
            else
            {
              blending = Ogre::SBT_REPLACE;
              depth_write = true;
            }

            std::set<Ogre::MaterialPtr>::iterator it;
            for(it = m_materials.begin(); it != m_materials.end(); it++)
            {
              Ogre::Technique* technique = (*it)->getTechnique( 0 );

              technique->setAmbient( c.r*0.5, c.g*0.5, c.b*0.5 );
              technique->setDiffuse( c.r, c.g, c.b, c.a );
              technique->setSceneBlending( blending );
              technique->setDepthWriteEnabled( depth_write );
              technique->setLightingEnabled( true );
            }
        }

        const std::string& getMeshResource() const {
            return m_meshResource;
        }

        // We are using this FrameListener callback to orient the mesh towards the camera.
        // Using a SceneManager::Listener and its preUpdateSceneGraph(SceneManager, Camera) method doesn't work because
        // it is apparently never invoked by the Rviz render system.
        virtual bool frameStarted(const Ogre::FrameEvent &evt)
        {
            Ogre::Quaternion cameraQuat;
            if(m_cameraFacing) {
                // Align with camera view direction
                // FIXME: The following way of retrieving the camera and its position is a bit hacky, don't try this at home!
                rviz::VisualizationManager* visualizationManager = dynamic_cast<rviz::VisualizationManager*>(m_displayContext);
                assert(visualizationManager != NULL);
                cameraQuat = visualizationManager->getRenderPanel()->getCamera()->getOrientation();
            }
            m_sceneNode->setOrientation(cameraQuat * m_orientation);
            return true;
        }

    private:
        Ogre::SceneManager* m_sceneManager;
        Ogre::SceneNode* m_sceneNode;
        rviz::DisplayContext* m_displayContext;

        Ogre::Quaternion m_orientation;
        Ogre::Entity* m_entity;
        std::set<Ogre::MaterialPtr> m_materials;
        std::string m_meshResource;
        bool m_cameraFacing;
    };

}

#endif // MESH_NODE_H
