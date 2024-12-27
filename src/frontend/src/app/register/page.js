"use client";
import React, { useState } from 'react';
import Image from 'next/image';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';

const Register = () => {
  const [fullname, setFullname] = useState('');
  const [crm, setCrm] = useState('');
  const [setor, setSetor] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');

  const handleRegister = async () => {
    if (password !== confirmPassword) {
      alert('As senhas não coincidem!');
      return;
    }

    try {
      const response = await fetch(
        `http://127.0.0.1:8000/usuario/criar?id=${crm}&nome=${fullname}&senha=${password}&setor=${setor}`,
        { method: 'POST' }
      );
      if (response.ok) {
        alert('Cadastro realizado com sucesso!');
      } else {
        alert('Erro ao cadastrar usuário.');
      }
    } catch (error) {
      alert('Erro ao conectar ao servidor.');
    }
  };

  return (
    <div className="h-screen grid grid-cols-1 md:grid-cols-5 font-sans">
      {/* Seção da Esquerda */}
      <div className="col-span-2 flex flex-col items-center justify-start bg-gray-100 text-center">
        <div>
          <Image width={500} height={500} src="/logo.png" alt="logo" />
        </div>
        <div className="flex items-start flex-col">
          <h2 className="text-5xl font-bold text-[#1282A2] mb-4 text-left">
            Bem Vindo de <br /> Volta!
          </h2>
          <p className="text-3xl font-bold text-[#1282A2] mb-16">Acesse sua conta!</p>
        </div>
        <a href='/login'>
          <Button className="rounded-full py-6 w-60 h-14 flex justify-center items-center text-xl bg-[#1282A2]">
            Entrar
          </Button>
        </a>
        <a href="#" className="mt-10 text-[#007b9f] hover:underline">
          Esqueceu a senha?
        </a>
      </div>

      {/* Seção da Direita */}
      <div className="col-span-3 flex-1 flex flex-col items-center justify-center bg-[#1282A2] text-white p-6">
        <h2 className="text-4xl font-bold mb-8">Registrar-se</h2>
        <div className="flex flex-col items-center space-y-6 w-3/4">
          <Input
            type="text"
            placeholder="Nome completo"
            value={fullname}
            onChange={(e) => setFullname(e.target.value)}
            className="text-black placeholder:text-gray-500 rounded-lg bg-gray-100 px-5 py-8 text-8xl h-10 border-none"
          />
          <Input
            type="text"
            placeholder="CRM"
            value={crm}
            onChange={(e) => setCrm(e.target.value)}
            className="text-black placeholder:text-gray-500 rounded-lg bg-gray-100 px-5 py-8 text-8xl h-10 border-none"
          />
          <Input
            type="text"
            placeholder="Setor"
            value={setor}
            onChange={(e) => setSetor(e.target.value)}
            className="text-black placeholder:text-gray-500 rounded-lg bg-gray-100 px-5 py-8 text-8xl h-10 border-none"
          />
          <Input
            type="password"
            placeholder="Senha"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            className="text-black placeholder:text-gray-500 rounded-lg bg-gray-100 px-5 py-8 text-8xl h-10 border-none"
          />
          <Input
            type="password"
            placeholder="Confirme a senha"
            value={confirmPassword}
            onChange={(e) => setConfirmPassword(e.target.value)}
            className="text-black placeholder:text-gray-500 rounded-lg bg-gray-100 px-5 py-8 text-8xl h-10 border-none"
          />
          <Button
            onClick={handleRegister}
            className="rounded-full py-6 w-60 h-14 flex justify-center items-center text-xl bg-gray-100 text-[#1282A2] hover:bg-[#1282A2] hover:text-black"
          >
            Cadastrar
          </Button>
        </div>
      </div>
    </div>
  );
};

export default Register;
