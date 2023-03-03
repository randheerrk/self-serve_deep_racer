import os
import json
import logging

from apache_atlas.utils           import type_coerce
from apache_atlas.model.misc      import SearchFilter
from apache_atlas.model.typedef   import AtlasTypesDef
from apache_atlas.model.instance  import AtlasEntityWithExtInfo
from apache_atlas.model.enums     import EntityOperation
from apache_atlas.utils           import API, BASE_URI,  HTTPStatus, HTTPMethod

LOG = logging.getLogger('AV Tech Data Catalogging')

class AtlasHelper :
    def __init__(self, client):
        self.client = client

    def createTypes(self) :
        with open("./json_files/typedefs.json") as json_file :
            typedef = type_coerce(json.load(json_file), AtlasTypesDef)
            self.typesDef = self.__create(typedef)

    def createEntity(self, entity_data):
        entity = type_coerce(entity_data, AtlasEntityWithExtInfo)
        try :
            response = self.client.entity.create_entity(entity)
            guid = None
            if response and response.mutatedEntities:
                if EntityOperation.CREATE.name in response.mutatedEntities:
                    header_list = response.mutatedEntities[EntityOperation.CREATE.name]
                elif EntityOperation.UPDATE.name in response.mutatedEntities:
                    header_list = response.mutatedEntities[EntityOperation.UPDATE.name]
                if header_list and len(header_list) > 0:
                    guid = header_list[0].guid
            elif response and response.guidAssignments:
                if entity.entity is not None and entity.entity.guid is not None:
                    in_guid = entity.entity.guid
                else:
                    in_guid = None
                if in_guid and response.guidAssignments[in_guid]:
                    guid = response.guidAssignments[in_guid]

            if guid:
                entity.entity.guid = guid

        except Exception as e:
            LOG.exception("failed to create entity %s. error=%s", entity, e)

        return entity.entity.guid if entity and entity.entity else None

    def deleteEntity(self, guids):
        try :
            self.client.entity.delete_entities_by_guids(guids)
            self.__purge_entities(guids)
        except Exception as e :
            LOG.exception("failed to delete entities")

    def __purge_entities(self, guids):
        uri = BASE_URI + "admin/purge/"
        api = API(uri, HTTPMethod.PUT, HTTPStatus.OK)
        try :
            self.client.call_api(api, request_obj=guids)
        except json.decoder.JSONDecodeError as e :
            LOG.exception("failed to purge with error %s", e)

    def __create(self, type_def):
        types_to_create = AtlasTypesDef()
        types_to_create.entityDefs = []
        types_to_create.relationshipDefs = []

        for entity_def in type_def.entityDefs:
            if self.client.typedef.type_with_name_exists(entity_def.name):
                LOG.info("Type with name %s already exists. Skipping.", entity_def.name)
            else:
                types_to_create.entityDefs.append(entity_def)

        for relationship_def in type_def.relationshipDefs :
            if self.client.typedef.type_with_name_exists(relationship_def.name):
                LOG.info("Type with name %s already exists. Skipping.", relationship_def.name)
            else:
                types_to_create.relationshipDefs.append(relationship_def)

        return self.client.typedef.create_atlas_typedefs(types_to_create)