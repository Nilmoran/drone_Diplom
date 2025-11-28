-- Создание базы данных
CREATE DATABASE microservice_db;

-- Подключение к базе
\c microservice_db;

-- Таблица пользователей
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    username VARCHAR(50) UNIQUE NOT NULL,
    email VARCHAR(100) UNIQUE NOT NULL,
    full_name VARCHAR(100),
    age INTEGER,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Таблица продуктов
CREATE TABLE products (
    id SERIAL PRIMARY KEY,
    name VARCHAR(100) NOT NULL,
    price DECIMAL(10, 2) NOT NULL,
    description TEXT,
    category VARCHAR(50),
    in_stock BOOLEAN DEFAULT true,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Вставка тестовых данных
INSERT INTO users (username, email, full_name, age) VALUES
('john_doe', 'john@example.com', 'John Doe', 30),
('jane_smith', 'jane@example.com', 'Jane Smith', 25),
('bob_wilson', 'bob@example.com', 'Bob Wilson', 35);

INSERT INTO products (name, price, description, category) VALUES
('Laptop', 999.99, 'High performance laptop', 'Electronics'),
('Smartphone', 499.99, 'Latest smartphone model', 'Electronics'),
('Book', 29.99, 'Programming book', 'Books'),
('Headphones', 79.99, 'Wireless headphones', 'Audio');

-- Создание индексов
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_products_category ON products(category);
CREATE INDEX idx_products_price ON products(price);