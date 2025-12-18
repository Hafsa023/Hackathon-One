"""
OAuth authentication service for social login providers.
Supports Google, GitHub, LinkedIn, and Twitter/X.
"""

from typing import Optional, Dict, Any
from authlib.integrations.starlette_client import OAuth
from starlette.config import Config
from sqlalchemy.orm import Session

from .config import get_settings
from .database import User
from .auth import create_access_token

settings = get_settings()

# OAuth client configuration
oauth = OAuth()

# Register OAuth providers (only if credentials are configured)
if settings.google_client_id and settings.google_client_secret:
    oauth.register(
        name='google',
        client_id=settings.google_client_id,
        client_secret=settings.google_client_secret,
        server_metadata_url='https://accounts.google.com/.well-known/openid-configuration',
        client_kwargs={'scope': 'openid email profile'},
    )

if settings.github_client_id and settings.github_client_secret:
    oauth.register(
        name='github',
        client_id=settings.github_client_id,
        client_secret=settings.github_client_secret,
        authorize_url='https://github.com/login/oauth/authorize',
        access_token_url='https://github.com/login/oauth/access_token',
        api_base_url='https://api.github.com/',
        client_kwargs={'scope': 'user:email'},
    )

if settings.linkedin_client_id and settings.linkedin_client_secret:
    oauth.register(
        name='linkedin',
        client_id=settings.linkedin_client_id,
        client_secret=settings.linkedin_client_secret,
        authorize_url='https://www.linkedin.com/oauth/v2/authorization',
        access_token_url='https://www.linkedin.com/oauth/v2/accessToken',
        api_base_url='https://api.linkedin.com/v2/',
        client_kwargs={'scope': 'openid profile email'},
    )

if settings.twitter_client_id and settings.twitter_client_secret:
    oauth.register(
        name='twitter',
        client_id=settings.twitter_client_id,
        client_secret=settings.twitter_client_secret,
        authorize_url='https://twitter.com/i/oauth2/authorize',
        access_token_url='https://api.twitter.com/2/oauth2/token',
        api_base_url='https://api.twitter.com/2/',
        client_kwargs={
            'scope': 'tweet.read users.read offline.access',
            'code_challenge_method': 'S256',
        },
    )


def get_enabled_providers() -> list:
    """Get list of enabled OAuth providers."""
    providers = []
    if settings.google_client_id and settings.google_client_secret:
        providers.append('google')
    if settings.github_client_id and settings.github_client_secret:
        providers.append('github')
    if settings.linkedin_client_id and settings.linkedin_client_secret:
        providers.append('linkedin')
    if settings.twitter_client_id and settings.twitter_client_secret:
        providers.append('twitter')
    return providers


def get_user_by_oauth(db: Session, provider: str, oauth_id: str) -> Optional[User]:
    """Get user by OAuth provider and ID."""
    return db.query(User).filter(
        User.oauth_provider == provider,
        User.oauth_id == oauth_id
    ).first()


def get_user_by_email(db: Session, email: str) -> Optional[User]:
    """Get user by email."""
    return db.query(User).filter(User.email == email).first()


def create_or_update_oauth_user(
    db: Session,
    provider: str,
    oauth_id: str,
    email: str,
    name: str,
    avatar_url: Optional[str] = None
) -> User:
    """Create or update a user from OAuth data."""
    # First check if user exists with this OAuth provider
    user = get_user_by_oauth(db, provider, oauth_id)

    if user:
        # Update existing OAuth user
        user.name = name
        if avatar_url:
            user.avatar_url = avatar_url
        db.commit()
        db.refresh(user)
        return user

    # Check if user exists with this email (might have registered with password)
    user = get_user_by_email(db, email)

    if user:
        # Link OAuth to existing account
        user.oauth_provider = provider
        user.oauth_id = oauth_id
        if avatar_url:
            user.avatar_url = avatar_url
        db.commit()
        db.refresh(user)
        return user

    # Create new user
    user = User(
        name=name,
        email=email,
        oauth_provider=provider,
        oauth_id=oauth_id,
        avatar_url=avatar_url,
        hashed_password=None,  # OAuth users don't have passwords
    )
    db.add(user)
    db.commit()
    db.refresh(user)
    return user


def create_token_for_user(user: User) -> str:
    """Create JWT token for a user."""
    return create_access_token(
        data={"sub": user.id, "email": user.email}
    )


async def get_google_user_info(token: dict) -> Dict[str, Any]:
    """Extract user info from Google OAuth token."""
    return {
        'oauth_id': token.get('userinfo', {}).get('sub'),
        'email': token.get('userinfo', {}).get('email'),
        'name': token.get('userinfo', {}).get('name'),
        'avatar_url': token.get('userinfo', {}).get('picture'),
    }


async def get_github_user_info(client, token: dict) -> Dict[str, Any]:
    """Fetch user info from GitHub API."""
    resp = await client.get('user', token=token)
    profile = resp.json()

    # Get primary email
    email_resp = await client.get('user/emails', token=token)
    emails = email_resp.json()
    primary_email = next(
        (e['email'] for e in emails if e.get('primary')),
        profile.get('email')
    )

    return {
        'oauth_id': str(profile.get('id')),
        'email': primary_email,
        'name': profile.get('name') or profile.get('login'),
        'avatar_url': profile.get('avatar_url'),
    }


async def get_linkedin_user_info(client, token: dict) -> Dict[str, Any]:
    """Fetch user info from LinkedIn API."""
    resp = await client.get('userinfo', token=token)
    profile = resp.json()

    return {
        'oauth_id': profile.get('sub'),
        'email': profile.get('email'),
        'name': profile.get('name'),
        'avatar_url': profile.get('picture'),
    }


async def get_twitter_user_info(client, token: dict) -> Dict[str, Any]:
    """Fetch user info from Twitter API."""
    resp = await client.get('users/me', token=token, params={
        'user.fields': 'id,name,username,profile_image_url'
    })
    data = resp.json().get('data', {})

    return {
        'oauth_id': data.get('id'),
        'email': None,  # Twitter doesn't provide email by default
        'name': data.get('name'),
        'avatar_url': data.get('profile_image_url'),
    }
