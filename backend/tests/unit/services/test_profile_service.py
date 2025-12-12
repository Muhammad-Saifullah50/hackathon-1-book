import pytest
from unittest.mock import AsyncMock, MagicMock
from uuid import uuid4
from src.services.profile_service import ProfileService
from src.models.profile import UserProfile, AgeRange, EducationLevel, TechBackground, PrimaryGoal, LearningMode, LearningSpeed
from src.models.auth import User # Assuming User model from auth module

class MockSupabaseClient:
    def __init__(self):
        self.table_data = {}
        self.current_table = None
        self.filter_column = None
        self.filter_value = None
        self.operation = None
        self.op_data = None
        self.is_single = False

    def table(self, table_name):
        self.current_table = table_name
        self.filter_column = None
        self.filter_value = None
        self.operation = None
        self.op_data = None
        self.is_single = False
        return self

    def insert(self, data):
        self.operation = 'insert'
        self.op_data = data
        return self

    def select(self, columns="*"):
        self.operation = 'select'
        return self

    def eq(self, column, value):
        self.filter_column = column
        self.filter_value = value
        return self

    def single(self):
        self.is_single = True
        return self

    def update(self, data):
        self.operation = 'update'
        self.op_data = data
        return self

    async def execute(self):
        if self.operation == 'insert':
            # Handle list vs single dict
            data_list = self.op_data if isinstance(self.op_data, list) else [self.op_data]
            # Deep copy to simulate storage
            stored_list = []
            for item in data_list:
                item_copy = item.copy()
                if "user_id" in item_copy:
                     item_copy["user_id"] = str(item_copy["user_id"])
                
                if self.current_table not in self.table_data:
                    self.table_data[self.current_table] = []
                self.table_data[self.current_table].append(item_copy)
                stored_list.append(item_copy)
            return MagicMock(data=stored_list, count=len(stored_list), status_code=201)

        elif self.operation == 'select':
            results = []
            if self.current_table in self.table_data:
                for item in self.table_data[self.current_table]:
                    if self.filter_column:
                        # Simple equality check
                        if str(item.get(self.filter_column)) == str(self.filter_value):
                            results.append(item)
                    else:
                        results.append(item)
            
            if self.is_single:
                if results:
                    return MagicMock(data=results[0], status_code=200)
                return MagicMock(data=None, status_code=404)
            return MagicMock(data=results, status_code=200)

        elif self.operation == 'update':
            results = []
            if self.current_table in self.table_data:
                for i, item in enumerate(self.table_data[self.current_table]):
                     if self.filter_column:
                        if str(item.get(self.filter_column)) == str(self.filter_value):
                            self.table_data[self.current_table][i].update(self.op_data)
                            results.append(self.table_data[self.current_table][i])
            return MagicMock(data=results, status_code=200)

        return MagicMock(data=None)


@pytest.fixture
def mock_supabase_client():
    return MockSupabaseClient()

@pytest.fixture
def profile_service(mock_supabase_client):
    return ProfileService(supabase_client=mock_supabase_client)

@pytest.mark.asyncio
async def test_create_profile_success(profile_service, mock_supabase_client):
    user_id = uuid4()
    profile_data = {
        "user_id": user_id,
        "age_range": AgeRange._18_24,
        "education_level": EducationLevel.UNDERGRAD,
        "tech_background": TechBackground.STUDENT,
        "primary_goal": PrimaryGoal.ACADEMIC,
        "learning_mode": LearningMode.TEXTUAL,
        "learning_speed": LearningSpeed.BALANCED,
        "time_per_week": 15,
        "preferred_language": "fr"
    }
    
    profile = await profile_service.create_profile(user_id=user_id, profile_data=profile_data)
    assert profile is not None
    assert profile.user_id == user_id
    assert mock_supabase_client.table_data["profiles"][0]["user_id"] == str(user_id)

@pytest.mark.asyncio
async def test_get_profile_success(profile_service, mock_supabase_client):
    user_id = uuid4()
    profile_data_to_insert = UserProfile(user_id=user_id, age_range=AgeRange._25_34).model_dump()
    # Manually insert data into mock
    if "profiles" not in mock_supabase_client.table_data:
        mock_supabase_client.table_data["profiles"] = []
    
    # Ensure user_id is string for mock matching
    data_copy = profile_data_to_insert.copy()
    data_copy["user_id"] = str(data_copy["user_id"])
    mock_supabase_client.table_data["profiles"].append(data_copy)

    profile = await profile_service.get_profile(user_id)
    assert profile is not None
    assert profile.user_id == user_id
    assert profile.age_range == AgeRange._25_34

@pytest.mark.asyncio
async def test_get_profile_not_found(profile_service):
    user_id = uuid4()
    profile = await profile_service.get_profile(user_id)
    assert profile is None

@pytest.mark.asyncio
async def test_update_profile_success(profile_service, mock_supabase_client):
    user_id = uuid4()
    profile_data_to_insert = UserProfile(user_id=user_id, age_range=AgeRange._18_24).model_dump()
    
    # Manually insert data into mock
    if "profiles" not in mock_supabase_client.table_data:
        mock_supabase_client.table_data["profiles"] = []
    
    data_copy = profile_data_to_insert.copy()
    data_copy["user_id"] = str(data_copy["user_id"])
    mock_supabase_client.table_data["profiles"].append(data_copy)

    update_data = {"time_per_week": 20, "learning_speed": LearningSpeed.INTENSIVE}
    updated_profile = await profile_service.update_profile(user_id, update_data)
    assert updated_profile is not None
    assert updated_profile.user_id == user_id
    assert updated_profile.time_per_week == 20
    assert updated_profile.learning_speed == LearningSpeed.INTENSIVE
    assert mock_supabase_client.table_data["profiles"][0]["time_per_week"] == 20
